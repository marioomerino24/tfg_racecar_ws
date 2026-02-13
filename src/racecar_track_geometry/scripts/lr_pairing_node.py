#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import rospy, tf
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point, PointStamped
from racecar_cone_msgs.msg import ConeArray, Cone, ConePair, ConePairArray

# ============== utilidades numéricas/TF ==============
def yaw_from_tf(rot):
    import tf.transformations as t
    e = t.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
    return e[2]

def dot(ax, ay, bx, by):
    return ax*bx + ay*by

def proj_var(S, vx, vy):
    # S = [xx, xy; yx, yy] con xy=yx
    return vx*(S[0]*vx + S[1]*vy) + vy*(S[2]*vx + S[3]*vy)

def get_cov_xy_safe(c):
    # Si tu msg no trae covarianza, usa un ruido modesto (~3 cm)
    try:
        S = c.cov_xy
        return [S[0], S[1], S[2], S[3]]
    except Exception:
        sig2 = (0.03*0.03)  # 3 cm^2
        return [sig2, 0.0, 0.0, sig2]


# ============ utilidades MST / componentes ============
class DisjointSet(object):
    """Union-Find sencillo para Kruskal."""
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank   = [0]*n

    def find(self, x):
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return False
        if self.rank[ra] < self.rank[rb]:
            self.parent[ra] = rb
        elif self.rank[ra] > self.rank[rb]:
            self.parent[rb] = ra
        else:
            self.parent[rb] = ra
            self.rank[ra] += 1
        return True


# =====================================================
class LRPairingNode(object):
    """
    Método MST + grafo bipartito umbralizado + matching máximo.
    """

    def __init__(self):
        ns = "~"

        # --- Tópicos ---
        self.topic_in   = rospy.get_param(ns+"topic_in",  "/cones/raw")
        self.topic_out  = rospy.get_param(ns+"topic_out", "/cones/pairs")
        self.topic_dbg  = rospy.get_param(ns+"topic_dbg", "/cones/pairs_debug")

        # === Parámetros geométricos de alto nivel ===
        self.track_width = float(rospy.get_param(ns + "track_width", 3.0))  # [m]
        self.cone_ds     = float(rospy.get_param(ns + "cone_ds",     1.0))  # [m]

        # === Parámetros de ancho de pista (EMA) ===
        self.w0    = self.track_width                     # ancho de pista estimado
        self.alpha = rospy.get_param(ns+"ema_w_alpha", 0.05)

        # === Umbral de corte para el MST (para separar bordes) ===
        self.mst_cut = rospy.get_param(ns+"mst_cut",
                                       0.5*(self.track_width + self.cone_ds))

        # === Parámetros grafo bipartito / matching máximo ===
        self.match_w_rel_min = rospy.get_param(ns+"match_w_rel_min", 0.99)  # *W0
        self.match_w_rel_max = rospy.get_param(ns+"match_w_rel_max", 1.01)  # *W0

        # === Estabilización temporal de parejas ===
        self.stab_enable         = rospy.get_param(ns+"stab/enable",        True)
        self.stab_em_max_factor  = rospy.get_param(ns+"stab/em_max_factor", 0.3)  # *DS
        self.stab_ew_max_rel     = rospy.get_param(ns+"stab/ew_max_rel",    0.2)  # *W0
        self.stab_beta_m         = rospy.get_param(ns+"stab/beta_m",        0.3)
        self.stab_beta_w         = rospy.get_param(ns+"stab/beta_w",        0.3)
        self.stab_s_behind_max   = rospy.get_param(ns+"stab/s_behind_max", -1.0)

        # === Límite de parejas hacia delante a publicar ===
        self.max_forward_pairs = int(rospy.get_param(ns + "max_forward_pairs", 5))

        # Margen por detrás del coche (s >= -s_back_margin)
        self.s_back_margin = float(rospy.get_param(ns + "s_back_margin", 0.2))

        # --- Filtro geométrico de deduplicado ---
        self.dedup_s_min = float(rospy.get_param(ns + "dedup_s_min",
                                                 0.3 * self.cone_ds))
        self.dedup_d_min = float(rospy.get_param(ns + "dedup_d_min",
                                                 0.3 * self.cone_ds))

        # Estado persistente de parejas estabilizadas
        self.prev_pairs = []

        # TF + IO
        self.tf = tf.TransformListener()
        self.pub_pairs = rospy.Publisher(self.topic_out, ConePairArray, queue_size=1)
        self.pub_debug = rospy.Publisher(self.topic_dbg, String,        queue_size=10)
        rospy.Subscriber(self.topic_in, ConeArray, self.cb, queue_size=1)

        rospy.loginfo("[lr_pairing_mst_graph] in=%s out=%s dbg=%s W=%.2f DS=%.2f mst_cut=%.2f",
                      self.topic_in, self.topic_out, self.topic_dbg,
                      self.track_width, self.cone_ds, self.mst_cut)

    # ------------------- ejes del robot en odom -------------------
    def base_frame_pose(self):
        try:
            self.tf.waitForTransform("odom", "base_link", rospy.Time(0),
                                     rospy.Duration(0.05))
            (trans, rot) = self.tf.lookupTransform("odom", "base_link",
                                                   rospy.Time(0))
            yaw  = yaw_from_tf(rot)
            pref = (trans[0], trans[1])
        except Exception:
            yaw  = 0.0
            pref = (0.0, 0.0)
        tx, ty = math.cos(yaw), math.sin(yaw)
        nx, ny = -math.sin(yaw), math.cos(yaw)
        return (tx, ty), (nx, ny), pref

    # -------------------------- helper ----------------------------
    def getP(self, c):
        p = getattr(c, "p", None)
        if p is None:
            p = getattr(c, "position", None)
        if p is None:
            p = Point(getattr(c, "x", 0.0),
                      getattr(c, "y", 0.0), 0.0)
        return p

    # -------------------- construcción MST ------------------------
    def build_mst(self, pts, debug_lines):
        N = len(pts)
        edges_all = []
        for i in range(N):
            xi, yi = pts[i]
            for j in range(i+1, N):
                xj, yj = pts[j]
                dx = xj - xi
                dy = yj - yi
                d  = math.hypot(dx, dy)
                edges_all.append((d, i, j))

        edges_all.sort(key=lambda e: e[0])

        ds = DisjointSet(N)
        edges_mst = []

        for (d, i, j) in edges_all:
            if ds.union(i, j):
                edges_mst.append((d, i, j))
                if len(edges_mst) == N-1:
                    break

        debug_lines.append("[3] MST construido: N=%d, aristas=%d" %
                           (N, len(edges_mst)))
        if len(edges_mst) > 0:
            d_min = edges_mst[0][0]
            d_max = edges_mst[-1][0]
            debug_lines.append("    longitudes MST: d_min=%.3f d_max=%.3f mst_cut=%.3f" %
                               (d_min, d_max, self.mst_cut))

        return edges_mst

    # ------------------ componentes (bordes) ----------------------
    def split_components(self, N, edges_mst, debug_lines):
        adj = [[] for _ in range(N)]
        cut_edges = []

        for (d, i, j) in edges_mst:
            if d <= self.mst_cut:
                adj[i].append(j)
                adj[j].append(i)
            else:
                cut_edges.append((d, i, j))

        debug_lines.append("[4] Corte MST en aristas largas:")
        debug_lines.append("    mst_cut=%.3f  #edges_short=%d  #edges_cut=%d" %
                           (self.mst_cut,
                            sum(1 for (d,_,_) in edges_mst if d <= self.mst_cut),
                            len(cut_edges)))
        if cut_edges:
            debug_lines.append("    primeras aristas cortadas (d, i, j): " +
                               ", ".join(["(%.2f,%d,%d)" % (d,i,j)
                                          for (d,i,j) in cut_edges[:5]]))

        visited = [False]*N
        comps = []

        for i in range(N):
            if visited[i]:
                continue
            stack = [i]
            comp = []
            visited[i] = True
            while stack:
                u = stack.pop()
                comp.append(u)
                for v in adj[u]:
                    if not visited[v]:
                        visited[v] = True
                        stack.append(v)
            comps.append(comp)

        debug_lines.append("    componentes encontradas: %d" % len(comps))
        for k, comp in enumerate(comps):
            debug_lines.append("    comp[%d]: size=%d indices=%s" %
                               (k, len(comp), str(comp)))

        return comps

    # ------------------ emparejado por grafo ----------------------
    def pair_incremental_local(self, msg, L_indices, R_indices,
                               pts_xy, t_robot, n_robot, pref, debug_lines):

        pairs = ConePairArray()
        pairs.header = Header(stamp=msg.header.stamp,
                              frame_id=msg.header.frame_id)

        if not L_indices or not R_indices:
            debug_lines.append("[7] Emparejado grafo: algún lado vacío, sin parejas.")
            debug_lines.append("    len(L_indices)=%d len(R_indices)=%d" %
                               (len(L_indices), len(R_indices)))
            return pairs

        def get_xy(idx):
            return pts_xy[idx]

        w0 = self.w0
        w_min = self.match_w_rel_min * w0
        w_max = self.match_w_rel_max * w0

        debug_lines.append("[7] Emparejado grafo bipartito umbralizado + matching máximo:")
        debug_lines.append("    W0 (EMA) = %.3f  match_w_rel = [%.2f, %.2f] -> w_min=%.3f w_max=%.3f" %
                           (w0,
                            self.match_w_rel_min, self.match_w_rel_max,
                            w_min, w_max))

        # --- [7.1] Construcción grafo ---
        adj = {}
        stats = {
            'total_pairs': 0,
            'too_small': 0,
            'too_big': 0,
            'accepted': 0
        }

        for iL in L_indices:
            xL, yL = get_xy(iL)
            for iR in R_indices:
                xR, yR = get_xy(iR)
                dx = xR - xL
                dy = yR - yL
                d  = math.hypot(dx, dy)

                stats['total_pairs'] += 1
                debug_lines.append(
                    "    [7.1] cand edge L=%d R=%d: dist=%.3f (w_min=%.3f, w_max=%.3f)" %
                    (iL, iR, d, w_min, w_max)
                )

                if d <= 1e-6:
                    debug_lines.append(
                        "           -> descarta por dist<=0 (d=%.6f)" % d
                    )
                    continue

                if d < w_min:
                    stats['too_small'] += 1
                    debug_lines.append(
                        "           -> descarta por dist < w_min (%.3f < %.3f)" % (d, w_min)
                    )
                    continue

                if d > w_max:
                    stats['too_big'] += 1
                    debug_lines.append(
                        "           -> descarta por dist > w_max (%.3f > %.3f)" % (d, w_max)
                    )
                    continue

                stats['accepted'] += 1
                debug_lines.append(
                    "           -> ACEPTA arista L=%d R=%d (dist=%.3f dentro ventana)" %
                    (iL, iR, d)
                )
                if iL not in adj:
                    adj[iL] = []
                adj[iL].append((iR, d))

        debug_lines.append("    [7.1] Estadísticas grafo bipartito:")
        debug_lines.append("           total_pairs=%d accepted=%d too_small=%d too_big=%d" %
                           (stats['total_pairs'], stats['accepted'],
                            stats['too_small'], stats['too_big']))

        if stats['accepted'] == 0 or not adj:
            debug_lines.append("    [7.1] Sin aristas válidas en el grafo -> 0 parejas.")
            return pairs

        # --- [7.2] Matching máximo (Kuhn) ---
        match_R = {}

        def dfs(iL, visited):
            if iL not in adj:
                return False
            neighbors = sorted(adj[iL], key=lambda e: abs(e[1] - self.w0))
            for (iR, d) in neighbors:
                if iR in visited:
                    continue
                visited.add(iR)
                owner = match_R.get(iR, None)
                if owner is None or dfs(owner, visited):
                    match_R[iR] = iL
                    debug_lines.append(
                        "        [match] asigna R=%d -> L=%d (dist=%.3f)" %
                        (iR, iL, d)
                    )
                    return True
            return False

        L_ordered = []
        for iL in L_indices:
            xL, yL = get_xy(iL)
            sL = (xL - pref[0])*t_robot[0] + (yL - pref[1])*t_robot[1]
            L_ordered.append((sL, iL))
        L_ordered.sort(key=lambda e: e[0])

        match_count = 0
        for (sL, iL) in L_ordered:
            visited = set()
            debug_lines.append("    [7.2] Intentando emparejar L=%d (sL=%.3f)" %
                               (iL, sL))
            if dfs(iL, visited):
                match_count += 1
                debug_lines.append("           -> L=%d emparejado." % iL)
            else:
                debug_lines.append("           -> L=%d sin pareja válida." % iL)

        debug_lines.append("    [7.3] Matching máximo resultante: n_matches=%d" %
                           match_count)

        if match_count == 0:
            debug_lines.append("    [7.3] Matching vacío -> 0 parejas.")
            return pairs

        # --- [7.4] Construcción lista parejas + prefijo continuo ---
        matches = []
        for iR, iL in match_R.items():
            xL, yL = get_xy(iL)
            xR, yR = get_xy(iR)
            mp_x = 0.5*(xL + xR)
            mp_y = 0.5*(yL + yR)
            width = math.hypot(xR - xL, yR - yL)
            s_mid = (mp_x - pref[0])*t_robot[0] + (mp_y - pref[1])*t_robot[1]
            matches.append((s_mid, iL, iR, mp_x, mp_y, width))

        matches.sort(key=lambda m: m[0])

        debug_lines.append("    [7.4] Parejas crudas (antes de recorte por continuidad):")
        for k, (s_mid, iL, iR, mp_x, mp_y, width) in enumerate(matches):
            debug_lines.append(
                "           raw_pair[%d]: s_mid=%.3f L=%d R=%d width=%.3f mp=(%.3f,%.3f)" %
                (k, s_mid, iL, iR, width, mp_x, mp_y)
            )

        L_seq = []
        for iL in L_indices:
            xL, yL = get_xy(iL)
            sL = (xL - pref[0])*t_robot[0] + (yL - pref[1])*t_robot[1]
            L_seq.append((sL, iL))
        L_seq.sort(key=lambda e: e[0])

        R_seq = []
        for iR in R_indices:
            xR, yR = get_xy(iR)
            sR = (xR - pref[0])*t_robot[0] + (yR - pref[1])*t_robot[1]
            R_seq.append((sR, iR))
        R_seq.sort(key=lambda e: e[0])

        pos_L = {}
        for idx, (sL, iL) in enumerate(L_seq):
            pos_L[iL] = idx
        pos_R = {}
        for idx, (sR, iR) in enumerate(R_seq):
            pos_R[iR] = idx

        debug_lines.append("    [7.4] Orden longitudinal L/R (s, idx):")
        debug_lines.append("           L_seq: " +
                           ", ".join(["(%.3f,%d)" % (sL, iL) for (sL, iL) in L_seq]))
        debug_lines.append("           R_seq: " +
                           ", ".join(["(%.3f,%d)" % (sR, iR) for (sR, iR) in R_seq]))

        filtered_matches = []
        last_pos_L = None
        last_pos_R = None

        for k, (s_mid, iL, iR, mp_x, mp_y, width) in enumerate(matches):
            pL = pos_L.get(iL, None)
            pR = pos_R.get(iR, None)

            if pL is None or pR is None:
                debug_lines.append(
                    "           WARNING: pareja[%d] L=%d R=%d sin posición en L_seq/R_seq, se omite." %
                    (k, iL, iR)
                )
                continue

            if last_pos_L is None:
                filtered_matches.append((s_mid, iL, iR, mp_x, mp_y, width))
                last_pos_L = pL
                last_pos_R = pR
                debug_lines.append(
                    "           [cont] pair[%d]: inicio prefijo continuo (pL=%d, pR=%d)" %
                    (k, pL, pR)
                )
                continue

            gap_L = pL - last_pos_L
            gap_R = pR - last_pos_R

            debug_lines.append(
                "           [cont] pair[%d]: pL=%d pR=%d gap_L=%d gap_R=%d" %
                (k, pL, pR, gap_L, gap_R)
            )

            if (gap_L > 1) or (gap_R > 1):
                debug_lines.append(
                    "           [cont] GAP detectado en pair[%d] (gap_L=%d, gap_R=%d) -> se corta." %
                    (k, gap_L, gap_R)
                )
                break

            filtered_matches.append((s_mid, iL, iR, mp_x, mp_y, width))
            last_pos_L = pL
            last_pos_R = pR

        debug_lines.append("    [7.4] Parejas tras recorte por continuidad: n=%d" %
                           len(filtered_matches))

        if not filtered_matches:
            debug_lines.append("    [7.4] Tras aplicar criterio de prefijo continuo no quedan parejas.")
            return pairs

        # --- [7.5] Construcción ConePairArray + actualización W0 ---
        sum_width = 0.0
        for idx_pair, (s_mid, iL, iR, mp_x, mp_y, width) in enumerate(filtered_matches):
            pair = ConePair()
            pair.header = pairs.header
            pair.left   = msg.cones[iL]
            pair.right  = msg.cones[iR]
            pair.midpoint = Point(mp_x, mp_y, 0.0)
            pair.width    = width
            pairs.pairs.append(pair)
            sum_width += width

            debug_lines.append(
                "    [7.5] pair[%d]: L=%d R=%d width=%.3f midpoint=(%.3f,%.3f) s_mid=%.3f" %
                (idx_pair, iL, iR, width, mp_x, mp_y, s_mid)
            )

        if len(filtered_matches) > 0:
            avg_w = sum_width / float(len(filtered_matches))
            old_w0 = self.w0
            self.w0 = (1.0 - self.alpha)*self.w0 + self.alpha*avg_w
            debug_lines.append(
                "    [7.6] EMA W0: old=%.3f avg_width=%.3f new=%.3f" %
                (old_w0, avg_w, self.w0)
            )

        return pairs

    # -------------- estabilización + orden final --------------
    def stabilize_and_order(self, pairs_prop, pref, t_robot, debug_lines):
        final_pairs = ConePairArray()
        final_pairs.header = Header(stamp=pairs_prop.header.stamp,
                                    frame_id=pairs_prop.header.frame_id)

        def clip_forward(entries_with_s):
            filtered = [(s, e) for (s, e) in entries_with_s
                        if s >= -self.s_back_margin]

            if self.max_forward_pairs <= 0:
                debug_lines.append(
                    "    [8] clip_forward: max_forward_pairs<=0 -> filtro sólo por s>=-%.3f, total=%d" %
                    (self.s_back_margin, len(filtered))
                )
                return filtered

            clipped = filtered[:self.max_forward_pairs]
            debug_lines.append(
                "    [8] clip_forward: max_pairs=%d back_margin=%.3f total_ok=%d -> kept=%d" %
                (self.max_forward_pairs, self.s_back_margin,
                 len(filtered), len(clipped))
            )
            return clipped

        def dedup_entries(entries, tag):
            """Filtro geométrico sencillo para eliminar parejas duplicadas."""
            if not entries:
                return entries
            result = [entries[0]]
            for i in range(1, len(entries)):
                s_curr, entry_curr = entries[i]
                s_last, entry_last = result[-1]
                ds = s_curr - s_last
                mp_curr = entry_curr['mp']
                mp_last = entry_last['mp']
                dxy = math.hypot(mp_curr[0] - mp_last[0],
                                 mp_curr[1] - mp_last[1])
                if (ds < self.dedup_s_min) or (dxy < self.dedup_d_min):
                    err_curr = abs(entry_curr['width'] - self.w0)
                    err_last = abs(entry_last['width'] - self.w0)
                    if err_curr < err_last:
                        result[-1] = (s_curr, entry_curr)
                        debug_lines.append(
                            "    [8] %s dedup: sustituye pareja duplicada (ds=%.3f dxy=%.3f)" %
                            (tag, ds, dxy)
                        )
                    else:
                        debug_lines.append(
                            "    [8] %s dedup: descarta pareja duplicada (ds=%.3f dxy=%.3f)" %
                            (tag, ds, dxy)
                        )
                else:
                    result.append((s_curr, entry_curr))
            debug_lines.append(
                "    [8] %s dedup: n_pairs antes=%d después=%d" %
                (tag, len(entries), len(result))
            )
            return result

        # --- Caso sin parejas propuestas ---
        if not pairs_prop.pairs:
            debug_lines.append("[8] Estabilización: sin parejas propuestas en este ciclo.")
            debug_lines.append("    prev_pairs disponibles=%d, stab_enable=%s" %
                               (len(self.prev_pairs), str(self.stab_enable)))
            if not self.stab_enable or not self.prev_pairs:
                debug_lines.append("    -> sin estado previo útil, salgo con 0 parejas.")
                self.prev_pairs = []
                return final_pairs

            debug_lines.append("    -> usando sólo estado previo (filtrando por s_behind_max=%.3f)." %
                               self.stab_s_behind_max)
            entries = []
            for k, entry in enumerate(self.prev_pairs):
                mp_x, mp_y = entry['mp']
                s_mid = (mp_x - pref[0])*t_robot[0] + (mp_y - pref[1])*t_robot[1]
                keep = (s_mid >= self.stab_s_behind_max)
                debug_lines.append("       prev_pair[%d]: mp=(%.3f,%.3f) width=%.3f s_mid=%.3f keep=%s" %
                                   (k, mp_x, mp_y, entry['width'], s_mid, str(keep)))
                if not keep:
                    continue
                entries.append((s_mid, entry))

            entries.sort(key=lambda x: x[0])
            entries = clip_forward(entries)
            entries = dedup_entries(entries, "prev-only")

            self.prev_pairs = []
            for idx_pair, (s_val, entry) in enumerate(entries):
                cp = ConePair()
                cp.header = final_pairs.header
                cp.left   = entry['left']
                cp.right  = entry['right']
                cp.midpoint = Point(entry['mp'][0], entry['mp'][1], 0.0)
                cp.width    = entry['width']
                try:
                    cp.left.id = idx_pair + 1
                except Exception:
                    pass
                try:
                    cp.right.id = idx_pair + 1
                except Exception:
                    pass
                final_pairs.pairs.append(cp)

                self.prev_pairs.append({
                    'left': entry['left'],
                    'right': entry['right'],
                    'mp':   entry['mp'],
                    'width': entry['width']
                })

            debug_lines.append("    -> parejas previas mantenidas: %d" %
                               len(final_pairs.pairs))
            return final_pairs

        # --- Caso sin estado previo o stab desactivada ---
        if (not self.stab_enable) or (not self.prev_pairs):
            debug_lines.append("[8] Estabilización: desactivada o sin estado previo.")
            entries = []
            for k, pair in enumerate(pairs_prop.pairs):
                mp = pair.midpoint
                mp_xy = (mp.x, mp.y)
                width = pair.width
                if width <= 0.0:
                    dx = pair.right.p.x - pair.left.p.x
                    dy = pair.right.p.y - pair.left.p.y
                    width = math.hypot(dx, dy)
                s_mid = (mp_xy[0]-pref[0])*t_robot[0] + (mp_xy[1]-pref[1])*t_robot[1]
                debug_lines.append(
                    "    cand_prop[%d]: mp=(%.3f,%.3f) width=%.3f s_mid=%.3f" %
                    (k, mp_xy[0], mp_xy[1], width, s_mid)
                )
                entries.append((s_mid, {
                    'left':  pair.left,
                    'right': pair.right,
                    'mp':    mp_xy,
                    'width': width
                }))

            entries.sort(key=lambda x: x[0])
            entries = clip_forward(entries)
            entries = dedup_entries(entries, "no-prev")

            self.prev_pairs = []
            for idx_pair, (s_val, entry) in enumerate(entries):
                cp = ConePair()
                cp.header = final_pairs.header
                cp.left   = entry['left']
                cp.right  = entry['right']
                cp.midpoint = Point(entry['mp'][0], entry['mp'][1], 0.0)
                cp.width    = entry['width']
                try:
                    cp.left.id = idx_pair + 1
                except Exception:
                    pass
                try:
                    cp.right.id = idx_pair + 1
                except Exception:
                    pass
                final_pairs.pairs.append(cp)

                self.prev_pairs.append({
                    'left':  entry['left'],
                    'right': entry['right'],
                    'mp':    entry['mp'],
                    'width': entry['width']
                })

            debug_lines.append("    -> n_pairs = %d (sin suavizado temporal)" %
                               len(final_pairs.pairs))
            return final_pairs

        # --- Estabilización completa ---
        prop_entries = []
        for k, pair in enumerate(pairs_prop.pairs):
            mp = pair.midpoint
            mp_xy = (mp.x, mp.y)
            width = pair.width
            if width <= 0.0:
                dx = pair.right.p.x - pair.left.p.x
                dy = pair.right.p.y - pair.left.p.y
                width = math.hypot(dx, dy)
            s_mid = (mp_xy[0]-pref[0])*t_robot[0] + (mp_xy[1]-pref[1])*t_robot[1]
            debug_lines.append(
                "    prop_pair_raw[%d]: mp=(%.3f,%.3f) width=%.3f s_mid=%.3f" %
                (k, mp_xy[0], mp_xy[1], width, s_mid)
            )
            prop_entries.append({
                'left':  pair.left,
                'right': pair.right,
                'mp':    mp_xy,
                'width': width,
                's':     s_mid
            })

        prev_entries = []
        for k, entry in enumerate(self.prev_pairs):
            mp_x, mp_y = entry['mp']
            s_mid = (mp_x - pref[0])*t_robot[0] + (mp_y - pref[1])*t_robot[1]
            debug_lines.append(
                "    prev_pair_raw[%d]: mp=(%.3f,%.3f) width=%.3f s_mid=%.3f" %
                (k, mp_x, mp_y, entry['width'], s_mid)
            )
            prev_entries.append({
                'left':  entry['left'],
                'right': entry['right'],
                'mp':    entry['mp'],
                'width': entry['width'],
                's':     s_mid
            })

        prop_entries.sort(key=lambda e: e['s'])
        prev_entries.sort(key=lambda e: e['s'])

        N_prev = len(prev_entries)
        N_prop = len(prop_entries)
        N_slots = max(N_prev, N_prop)

        e_m_max = self.stab_em_max_factor * self.cone_ds
        e_w_max = self.stab_ew_max_rel * max(self.w0, 1e-3)

        debug_lines.append("[8] Estabilización temporal completa:")
        debug_lines.append("    prev=%d prop=%d em_max=%.3f ew_max=%.3f s_behind_max=%.3f" %
                           (N_prev, N_prop, e_m_max, e_w_max, self.stab_s_behind_max))

        new_entries = []

        for idx in range(N_slots):
            if idx < N_prev and idx < N_prop:
                prev = prev_entries[idx]
                prop = prop_entries[idx]

                dx = prop['mp'][0] - prev['mp'][0]
                dy = prop['mp'][1] - prev['mp'][1]
                e_m = math.hypot(dx, dy)
                e_w = abs(prop['width'] - prev['width'])

                debug_lines.append(
                    "    slot[%d]: prev_s=%.3f prop_s=%.3f prev_width=%.3f prop_width=%.3f e_m=%.3f e_w=%.3f" %
                    (idx, prev['s'], prop['s'], prev['width'], prop['width'], e_m, e_w)
                )

                if (e_m <= e_m_max) and (e_w <= e_w_max):
                    mp_x_new = (1.0 - self.stab_beta_m)*prev['mp'][0] + \
                               self.stab_beta_m*prop['mp'][0]
                    mp_y_new = (1.0 - self.stab_beta_m)*prev['mp'][1] + \
                               self.stab_beta_m*prop['mp'][1]
                    w_new = (1.0 - self.stab_beta_w)*prev['width'] + \
                            self.stab_beta_w*prop['width']

                    new_entries.append({
                        'left':  prop['left'],
                        'right': prop['right'],
                        'mp':    (mp_x_new, mp_y_new),
                        'width': w_new
                    })
                    debug_lines.append("        -> ACEPTA+SUAVIZA (usa conos nuevos).")
                else:
                    if prev['s'] > self.stab_s_behind_max:
                        new_entries.append({
                            'left':  prev['left'],
                            'right': prev['right'],
                            'mp':    prev['mp'],
                            'width': prev['width']
                        })
                        debug_lines.append("        -> NO ACTUALIZA (mantiene pareja previa).")
                    else:
                        new_entries.append({
                            'left':  prop['left'],
                            'right': prop['right'],
                            'mp':    prop['mp'],
                            'width': prop['width']
                        })
                        debug_lines.append("        -> REEMPLAZA: previa muy atrás, usa propuesta.")
            elif idx < N_prop:
                prop = prop_entries[idx]
                new_entries.append({
                    'left':  prop['left'],
                    'right': prop['right'],
                    'mp':    prop['mp'],
                    'width': prop['width']
                })
                debug_lines.append("    slot[%d]: NUEVA pareja añadida (s=%.3f)." %
                                   (idx, prop['s']))
            else:
                prev = prev_entries[idx]
                if prev['s'] > self.stab_s_behind_max:
                    new_entries.append({
                        'left':  prev['left'],
                        'right': prev['right'],
                        'mp':    prev['mp'],
                        'width': prev['width']
                    })
                    debug_lines.append("    slot[%d]: Mantiene pareja previa sin propuesta (s=%.3f)." %
                                       (idx, prev['s']))
                else:
                    debug_lines.append("    slot[%d]: Elimina pareja previa (muy atrás, s=%.3f)." %
                                       (idx, prev['s']))

        entries_with_s = []
        for entry in new_entries:
            mp_x, mp_y = entry['mp']
            s_mid = (mp_x - pref[0])*t_robot[0] + (mp_y - pref[1])*t_robot[1]
            entries_with_s.append((s_mid, entry))
        entries_with_s.sort(key=lambda e: e[0])
        entries_with_s = clip_forward(entries_with_s)
        entries_with_s = dedup_entries(entries_with_s, "stab")

        self.prev_pairs = []
        for idx_pair, (s_val, entry) in enumerate(entries_with_s):
            cp = ConePair()
            cp.header = final_pairs.header
            cp.left   = entry['left']
            cp.right  = entry['right']
            cp.midpoint = Point(entry['mp'][0], entry['mp'][1], 0.0)
            cp.width    = entry['width']
            try:
                cp.left.id = idx_pair + 1
            except Exception:
                pass
            try:
                cp.right.id = idx_pair + 1
            except Exception:
                pass
            final_pairs.pairs.append(cp)

            self.prev_pairs.append({
                'left':  entry['left'],
                'right': entry['right'],
                'mp':    entry['mp'],
                'width': entry['width']
            })

        debug_lines.append("    -> n_pairs_final = %d tras estabilización." %
                           len(final_pairs.pairs))

        return final_pairs

    # --------------------- callback principal ---------------------
    def cb(self, msg):
        debug_lines = []

        t_msg = msg.header.stamp.to_sec()
        frame = msg.header.frame_id

        debug_lines.append("========== lr_pairing MST+GRAPH DEBUG ==========")
        debug_lines.append("t = %.6f  frame = %s" % (t_msg, frame))
        debug_lines.append("track_width W = %.3f  cone_ds DS = %.3f" %
                           (self.track_width, self.cone_ds))
        debug_lines.append("W0 (estado previo EMA) = %.3f" % self.w0)
        debug_lines.append("Parámetros estabilización: enable=%s em_max_factor=%.3f ew_max_rel=%.3f "
                           "beta_m=%.3f beta_w=%.3f s_behind_max=%.3f" %
                           (str(self.stab_enable),
                            self.stab_em_max_factor,
                            self.stab_ew_max_rel,
                            self.stab_beta_m,
                            self.stab_beta_w,
                            self.stab_s_behind_max))

        debug_lines.append("Parámetros matching grafo: match_w_rel=[%.3f, %.3f]" %
                           (self.match_w_rel_min, self.match_w_rel_max))

        # [1] Entrada
        debug_lines.append("[1] ConeArray de entrada:")
        N = len(msg.cones)
        debug_lines.append("    n_cones = %d" % N)

        if N < 2:
            debug_lines.append("    Muy pocos conos para construir MST / emparejar.")
            pairs_empty = ConePairArray()
            pairs_empty.header = Header(stamp=msg.header.stamp,
                                        frame_id=msg.header.frame_id)
            dbg_msg = String()
            dbg_msg.data = "\n".join(debug_lines)
            self.pub_debug.publish(dbg_msg)
            self.pub_pairs.publish(pairs_empty)
            return

        pts_xy = []
        for i, c in enumerate(msg.cones):
            p = self.getP(c)
            pts_xy.append((p.x, p.y))
            debug_lines.append(
                "    cone[%3d]: id=%d  p=(%.3f, %.3f, %.3f) side_field=%d conf=%.2f src=%s" %
                (i, getattr(c, "id", -1),
                 p.x, p.y, p.z,
                 getattr(c, "side", Cone.UNKNOWN),
                 getattr(c, "confidence", 0.0),
                 getattr(c, "source", ""))
            )

        # [2] Pose robot
        t_robot, n_robot, pref = self.base_frame_pose()
        debug_lines.append("[2] Pose del robot y ejes en ODOM:")
        debug_lines.append("    pref (robot) = (%.3f, %.3f)" % (pref[0], pref[1]))
        debug_lines.append("    t (avance)   = (%.3f, %.3f)" %
                           (t_robot[0], t_robot[1]))
        debug_lines.append("    n (lateral)  = (%.3f, %.3f)" %
                           (n_robot[0], n_robot[1]))

        # [3] MST
        edges_mst = self.build_mst(pts_xy, debug_lines)

        # [4] Componentes
        comps = self.split_components(N, edges_mst, debug_lines)

        if len(comps) < 2:
            debug_lines.append("    WARNING: <2 componentes principales. Fallback L/R por lateral.")
            L_indices = []
            R_indices = []
            for i, (x, y) in enumerate(pts_xy):
                relx = x - pref[0]
                rely = y - pref[1]
                lat  = relx*n_robot[0] + rely*n_robot[1]
                if lat >= 0.0:
                    L_indices.append(i)
                    debug_lines.append(
                        "        cone %d: lat=%.3f -> L (fallback)" % (i, lat)
                    )
                else:
                    R_indices.append(i)
                    debug_lines.append(
                        "        cone %d: lat=%.3f -> R (fallback)" % (i, lat)
                    )
        else:
            comps_sorted = sorted(comps, key=lambda c: len(c), reverse=True)
            compA = comps_sorted[0]
            compB = comps_sorted[1]

            debug_lines.append("    Tomamos dos componentes mayores como bordes:")
            debug_lines.append("        compA size=%d indices=%s" %
                               (len(compA), str(compA)))
            debug_lines.append("        compB size=%d indices=%s" %
                               (len(compB), str(compB)))

            def mean_lateral(comp):
                if len(comp) == 0:
                    return 0.0
                s = 0.0
                for idx in comp:
                    x, y = pts_xy[idx]
                    relx = x - pref[0]
                    rely = y - pref[1]
                    s += relx*n_robot[0] + rely*n_robot[1]
                return s / float(len(comp))

            latA = mean_lateral(compA)
            latB = mean_lateral(compB)
            debug_lines.append("    medias laterales: compA=%.3f compB=%.3f" %
                               (latA, latB))

            if latA >= latB:
                L_indices = compA
                R_indices = compB
            else:
                L_indices = compB
                R_indices = compA

        debug_lines.append("[5] Indices clasificados por MST en L/R:")
        debug_lines.append("    L_indices=%s" % str(sorted(L_indices)))
        debug_lines.append("    R_indices=%s" % str(sorted(R_indices)))

        # [6] Orden L/R por s (sólo debug)
        L_dbg = []
        R_dbg = []
        for i in L_indices:
            x, y = pts_xy[i]
            s = (x - pref[0])*t_robot[0] + (y - pref[1])*t_robot[1]
            L_dbg.append((s, i))
        for i in R_indices:
            x, y = pts_xy[i]
            s = (x - pref[0])*t_robot[0] + (y - pref[1])*t_robot[1]
            R_dbg.append((s, i))
        L_dbg.sort(key=lambda x: x[0])
        R_dbg.sort(key=lambda x: x[0])

        debug_lines.append("[6] Bordes L/R ordenados por s (robot) [solo debug]:")
        debug_lines.append("    L (s, idx_msg): " +
                           ", ".join(["(%.3f,%d)" % (sL, iL) for (sL, iL) in L_dbg]))
        debug_lines.append("    R (s, idx_msg): " +
                           ", ".join(["(%.3f,%d)" % (sR, iR) for (sR, iR) in R_dbg]))

        # [7] Emparejado
        pairs_prop = self.pair_incremental_local(msg, L_indices, R_indices,
                                                 pts_xy, t_robot, n_robot, pref,
                                                 debug_lines)

        # [8] Estabilización
        pairs = self.stabilize_and_order(pairs_prop, pref, t_robot, debug_lines)

        # [9] Resultado final
        debug_lines.append("[9] ConePairArray publicado en %s:" % self.topic_out)
        debug_lines.append("     n_pairs = %d" % len(pairs.pairs))
        for i, pair in enumerate(pairs.pairs):
            mp  = pair.midpoint
            pid = getattr(pair.left, "id", -1)
            debug_lines.append(
                "    pair[%3d]: id=%d width=%.3f midpoint=(%.3f, %.3f)" %
                (i, pid, pair.width, mp.x, mp.y)
            )

        debug_lines.append("========== end lr_pairing MST+GRAPH DEBUG ==========")

        dbg_msg = String()
        dbg_msg.data = "\n".join(debug_lines)
        self.pub_debug.publish(dbg_msg)
        self.pub_pairs.publish(pairs)


# =====================================================
if __name__ == "__main__":
    rospy.init_node("lr_pairing_mst_graph", anonymous=False)
    LRPairingNode()
    rospy.spin()

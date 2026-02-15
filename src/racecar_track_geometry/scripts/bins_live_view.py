#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import re
import sys
import rospy
from std_msgs.msg import String


class BinsLiveView(object):
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/estimation/track/midpoints/debug")
        self.include_empty_bins = bool(rospy.get_param("~include_empty_bins", False))
        self.clear_screen = bool(rospy.get_param("~clear_screen", True))

        self.re_seq = re.compile(r"seq=(\d+).*t=([0-9.]+)")
        self.re_bin = re.compile(r"^- bin\s+(-?\d+)")
        self.re_cones = re.compile(r"L=(\[[^\]]*\]),\s*R=(\[[^\]]*\])")
        self.re_decision = re.compile(r"decision=([a-zA-Z0-9_]+)")
        self.re_inferred = re.compile(r"inferred=([a-zA-Z]+)")

        rospy.Subscriber(self.topic, String, self.cb, queue_size=1)
        rospy.loginfo("bins_live_view: escuchando %s", self.topic)

    def _print_header(self, seq, stamp):
        if self.clear_screen:
            # Limpia terminal para vista "dashboard" en vivo
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()
        print("=== BINS LIVE VIEW ===")
        print("seq=%s  t=%s" % (seq, stamp))
        print("")
        print("%-6s | %-18s | %-18s | %-36s | %-8s" %
              ("bin", "L idx", "R idx", "decision", "inferred"))
        print("-" * 98)

    def cb(self, msg):
        txt = msg.data if msg.data else ""
        lines = txt.splitlines()
        if not lines:
            return

        seq = "?"
        stamp = "?"
        m = self.re_seq.search(lines[0])
        if m:
            seq = m.group(1)
            stamp = m.group(2)

        in_bins = False
        rows = []
        cur = None
        for ln in lines:
            if "===== 10) PROCESO POR BIN =====" in ln:
                in_bins = True
                cur = None
                continue
            if in_bins and ln.startswith("===== 11)"):
                if cur is not None:
                    rows.append(cur)
                break
            if not in_bins:
                continue

            mb = self.re_bin.search(ln)
            if mb:
                if cur is not None:
                    rows.append(cur)
                cur = {
                    "bin": mb.group(1),
                    "L": "[]",
                    "R": "[]",
                    "decision": "n/a",
                    "inferred": "n/a",
                }
                continue

            if cur is None:
                continue

            mc = self.re_cones.search(ln)
            if mc:
                cur["L"] = mc.group(1)
                cur["R"] = mc.group(2)
                continue

            md = self.re_decision.search(ln)
            if md:
                cur["decision"] = md.group(1)
                continue

            mi = self.re_inferred.search(ln)
            if mi:
                cur["inferred"] = mi.group(1)
                continue

        if cur is not None and (len(rows) == 0 or rows[-1] is not cur):
            rows.append(cur)

        shown = 0
        self._print_header(seq, stamp)
        for r in rows:
            if (not self.include_empty_bins) and r["L"] == "[]" and r["R"] == "[]":
                continue
            shown += 1
            print("%-6s | %-18s | %-18s | %-36s | %-8s" %
                  (r["bin"], r["L"], r["R"], r["decision"], r["inferred"]))

        if shown == 0:
            print("(sin bins activos en este frame)")


if __name__ == "__main__":
    rospy.init_node("bins_live_view")
    BinsLiveView()
    rospy.spin()

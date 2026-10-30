"""Mark tags of the drive log (log version 2) - same codes as DriveLog.h ("Mark TAGS").

tag byte:  bits 0-3 feature that ends the segment, bits 4-5 side, bits 6-7 how the segment was driven.
"""
FEATURE = {0: "note", 1: "wall ends", 2: "wall starts", 3: "wall/object", 4: "ramp starts",
           5: "ramp ends", 6: "object beside", 7: "corner", 8: "open area", 14: "start", 15: "undo"}
SIDE = {0: "", 1: "left", 2: "right", 3: "ahead"}
DRIVE = {0: "", 1: "L wall", 2: "R wall", 3: "heading"}
START, UNDO, NOTE = 14, 15, 0


def split(tag):
    tag = int(tag)
    return tag & 15, (tag >> 4) & 3, (tag >> 6) & 3


def text(tag):
    f, s, d = split(tag)
    if f in (START, UNDO):
        return FEATURE[f]
    out = (DRIVE[d] + " | " if DRIVE[d] else "") + FEATURE.get(f, f"feature {f}")
    return out + (" " + SIDE[s] if SIDE[s] else "")


def presses(marks, tags):
    """[(record index, mark number, tag)] of every Mark press, with UNDO presses and the presses they
    cancel removed. marks / tags: per-record mark count and tag of the latest mark."""
    out = []
    for i in range(1, len(marks)):
        if marks[i] > marks[i - 1]:
            t = int(tags[i])
            if t & 15 == UNDO:
                if out:
                    out.pop()
            else:
                out.append((i, int(marks[i]), t))
    return out

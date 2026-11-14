"""Trace GARBY's existing wordmark/cart into the adaptive icon's guaranteed safe area.

Run with Python and Pillow. The original artwork is retained unchanged. This produces
an Android vector resource, removing the print-only ring/tagline from the launcher.
"""
from pathlib import Path
from math import hypot

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "design/garby_logo_source_2000x1467.webp"
OUTPUT = ROOT / "app/src/main/res/drawable/ic_launcher_foreground.xml"


def simplify(points, tolerance=1.4):
    if len(points) < 3:
        return points
    x0, y0 = points[0]
    x1, y1 = points[-1]
    dx, dy = x1 - x0, y1 - y0
    length = hypot(dx, dy)
    distances = [
        abs(dy * (x - x0) - dx * (y - y0)) / length if length else hypot(x - x0, y - y0)
        for x, y in points
    ]
    split = max(range(len(points)), key=distances.__getitem__)
    if distances[split] <= tolerance:
        return [points[0], points[-1]]
    return simplify(points[:split + 1], tolerance)[:-1] + simplify(points[split:], tolerance)


def trace(image, region, target):
    crop = image.crop(region)
    width, height = crop.size
    read_pixels = getattr(crop, "get_flattened_data", crop.getdata)
    pixels = [a >= 128 and min(r, g, b) >= 235 for r, g, b, a in read_pixels()]
    edges = {}

    def solid(x, y):
        return 0 <= x < width and 0 <= y < height and pixels[y * width + x]

    def edge(start, end):
        edges.setdefault(start, []).append(end)

    for y in range(height):
        for x in range(width):
            if not solid(x, y):
                continue
            if not solid(x, y - 1):
                edge((x, y), (x + 1, y))
            if not solid(x + 1, y):
                edge((x + 1, y), (x + 1, y + 1))
            if not solid(x, y + 1):
                edge((x + 1, y + 1), (x, y + 1))
            if not solid(x - 1, y):
                edge((x, y + 1), (x, y))

    contours = []
    while edges:
        start = next(iter(edges))
        cursor = start
        contour = []
        while True:
            contour.append(cursor)
            adjacent = edges[cursor]
            following = adjacent.pop()
            if not adjacent:
                del edges[cursor]
            cursor = following
            if cursor == start:
                break
        area = abs(sum(
            x * ny - nx * y
            for (x, y), (nx, ny) in zip(contour, contour[1:] + contour[:1])
        )) / 2
        if area < 24:
            continue
        split = max(range(len(contour)), key=lambda i: hypot(
            contour[i][0] - start[0], contour[i][1] - start[1]
        ))
        contours.append(
            simplify(contour[:split + 1])[:-1] + simplify(contour[split:] + [start])[:-1]
        )

    x0 = min(x for contour in contours for x, _ in contour)
    x1 = max(x for contour in contours for x, _ in contour)
    y0 = min(y for contour in contours for _, y in contour)
    scale = target[2] / (x1 - x0)
    return [[(target[0] + (x - x0) * scale, target[1] + (y - y0) * scale)
             for x, y in contour] for contour in contours]


def main():
    image = Image.open(SOURCE).convert("RGBA")
    marks = [
        trace(image, (450, 430, 1450, 725), (26, 37.8, 56)),
        trace(image, (490, 805, 1415, 1120), (26, 57, 56)),
    ]
    radius = max(hypot(x - 54, y - 54) for mark in marks for contour in mark for x, y in contour)
    scale = min(1.0, 32.4 / radius)
    paths = []
    for mark in marks:
        parts = []
        for contour in mark:
            points = [(54 + (x - 54) * scale, 54 + (y - 54) * scale) for x, y in contour]
            parts.append("M" + "L".join(f"{x:.3f},{y:.3f}" for x, y in points) + "Z")
        paths.append("".join(parts))
    xml = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<!-- Official GARBY wordmark and cart, traced from the retained source artwork.',
        '     Print-only ring/tagline omitted. All paths fit inside the 66 dp safe circle. -->',
        '<vector xmlns:android="http://schemas.android.com/apk/res/android"',
        '    android:width="108dp" android:height="108dp"',
        '    android:viewportWidth="108" android:viewportHeight="108">',
    ]
    xml += [f'    <path android:fillColor="#FFFAFAFA" android:fillType="evenOdd" android:pathData="{path}" />'
            for path in paths]
    xml.append('</vector>')
    OUTPUT.write_text("\n".join(xml) + "\n", encoding="utf-8")
    print(f"Wrote {OUTPUT.relative_to(ROOT)}; mark radius {radius * scale:.2f} dp (safe limit 33 dp)")


if __name__ == "__main__":
    main()

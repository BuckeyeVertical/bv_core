from PIL import Image, ImageChops
import sys

if len(sys.argv) != 4:
    print("Usage: python layer_images.py <bottom_image> <top_image> <output_image>")
    sys.exit(1)

bottom_path = sys.argv[1]
top_path = sys.argv[2]
output_path = sys.argv[3]

FEATHER_SIZE = 250  # set to 0 to disable feathering

bottom = Image.open(bottom_path).convert("RGBA")
top = Image.open(top_path).convert("RGBA")

print("Original bottom size:", bottom.size)
print("Original top size:", top.size)

# Shared target box: average of the two image sizes
target_w = int((bottom.width + top.width) / 2)
target_h = int((bottom.height + top.height) / 2)

def resize_to_fit(img, target_w, target_h):
    scale = min(target_w / img.width, target_h / img.height)
    new_w = int(img.width * scale)
    new_h = int(img.height * scale)
    return img.resize((new_w, new_h), Image.Resampling.LANCZOS)

# Resize both images to fit inside the same target box
bottom_resized = resize_to_fit(bottom, target_w, target_h)
top_resized = resize_to_fit(top, target_w, target_h)

print("Resized bottom size:", bottom_resized.size)
print("Resized top size:", top_resized.size)

# Final canvas is the shared target box
canvas_w = target_w
canvas_h = target_h

bottom_canvas = Image.new("RGBA", (canvas_w, canvas_h), (0, 0, 0, 0))
top_canvas = Image.new("RGBA", (canvas_w, canvas_h), (0, 0, 0, 0))

# Center both images on the same point
bottom_x = (canvas_w - bottom_resized.width) // 2
bottom_y = (canvas_h - bottom_resized.height) // 2

top_x = (canvas_w - top_resized.width) // 2
top_y = (canvas_h - top_resized.height) // 2

bottom_canvas.paste(bottom_resized, (bottom_x, bottom_y), bottom_resized)
top_canvas.paste(top_resized, (top_x, top_y), top_resized)

# Optional feathering on the top image
if FEATHER_SIZE > 0:
    original_alpha = top_canvas.getchannel("A")
    edge_mask = Image.new("L", (canvas_w, canvas_h), 255)
    pixels = edge_mask.load()

    for y in range(canvas_h):
        for x in range(canvas_w):
            distance = min(
                x,
                y,
                canvas_w - 1 - x,
                canvas_h - 1 - y
            )

            if distance < FEATHER_SIZE:
                t = distance / FEATHER_SIZE
                t = t * t * (3 - 2 * t)  # smoothstep
                pixels[x, y] = int(255 * t)

    combined_alpha = ImageChops.multiply(original_alpha, edge_mask)
    top_canvas.putalpha(combined_alpha)

# Composite top over bottom
result = Image.alpha_composite(bottom_canvas, top_canvas)
result.save(output_path)

print("Saved:", output_path)

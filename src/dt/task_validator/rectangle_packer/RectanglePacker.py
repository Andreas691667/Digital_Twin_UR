import rpack

class RectanglePacker:
    def __init__(self, container_width, container_height):
        self.container_width = container_width
        self.container_height = container_height

    def pack(self, rectangles, block_height=4, block_width=4, padding=1):
        rectangles_padded = [
            (x[0] + padding, x[1] + padding) for x in rectangles
        ]
        packed_rectangles = rpack.pack(
            rectangles_padded, max_height=self.container_height, max_width=self.container_width
        )
        return packed_rectangles


# # Define your container size
# container_width = 4 * 4
# container_height = 4 * 4
# delta = 0
# # Create a RectanglePacker instance with the container size
# # packer = RectanglePacker(container_width, container_height)

# # Define the rectangles you want to pack
# # rectangles = [(5,6), (5,5), (4,4), (5,5), (4,5), (5,6)]
# # 8 rectanfles (2,2)
# # rectangles = [(2, 2), (2, 2), (2, 2), (2, 2), (2,2), (2,2), (2,2), (2,2), (3,3)]
# # 100 rectangles (2,2)
# # rectangles = [(4, 4)]*8
# rectangles = [(4, 4), (8, 8)]
# rectangles_padded = [(x[0] + delta, x[1] + delta) for x in rectangles]

# # Pack the rectangles into the container
# packed_rectangles = rpack.pack(
#     rectangles_padded, max_height=container_height, max_width=container_width
# )
# print(packed_rectangles)


# # # Visualize the packed rectangles
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches

# fig, ax = plt.subplots()
# ax.set_xlim(0, container_width)
# ax.set_ylim(0, container_height)
# for i, rect in enumerate(packed_rectangles):
#     x = rect[0]
#     y = rect[1]
#     width = rectangles[i][0]
#     height = rectangles[i][1]
#     ax.add_patch(patches.Rectangle((x, y), width, height, fill=False))
#     # number each rectangle
#     ax.text(x + width / 2, y + height / 2, str(i), ha="center", va="center")
# # equal aspect ratio
# ax.set_aspect("equal", "box")
# plt.show()

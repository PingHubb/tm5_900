# import pygame
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
#
# # Constants
# WIDTH, HEIGHT = 800, 600
# COLS, ROWS = 10, 11
# CELL_SIZE = min(WIDTH // COLS, HEIGHT // ROWS)
# WHITE = (255, 255, 255)
# BLACK = (0, 0, 0)
# GREY = (200, 200, 200)
# FONT_SIZE = CELL_SIZE // 2  # Adjust font size as needed
#
# # Initialize Pygame
# pygame.init()
# win = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("11x10 Matrix")
# font = pygame.font.SysFont(None, FONT_SIZE)  # Initialize font
#
# # ROS 2 Node and Publisher
# class MatrixPublisher(Node):
#     def __init__(self):
#         super().__init__('my_send_list')
#         self.publisher = self.create_publisher(Float32MultiArray, 'heatmap_data_topic', 10)
#         self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
#         self.matrix = [[0 for _ in range(COLS)] for _ in range(ROWS)]
#         self.hovered_cell = None
#         self.hover_start_time = None
#
#     def timer_callback(self):
#         self.publish_list()
#
#     def publish_list(self):
#         flat_matrix = [cell for row in self.matrix for cell in row]
#         msg = Float32MultiArray(data=flat_matrix)
#         self.publisher.publish(msg)
#         # Print the matrix data to the terminal
#         print(f"Published matrix data: {flat_matrix}")
#
#     def update_hovered_cell(self, row, col, is_hovering):
#         if is_hovering:
#             if self.hovered_cell != (row, col):
#                 self.hovered_cell = (row, col)
#                 self.hover_start_time = pygame.time.get_ticks()
#         else:
#             self.hovered_cell = None
#             self.hover_start_time = None
#
#     def increment_hovered_cell_value(self):
#         if self.hovered_cell is not None and self.hover_start_time is not None:
#             elapsed_time = (pygame.time.get_ticks() - self.hover_start_time) / 1000  # Convert milliseconds to seconds
#             increment = min(2000, (elapsed_time / 5) * 2000)
#             row, col = self.hovered_cell
#             self.matrix[row][col] = increment
#         elif self.hovered_cell is None:
#             for i in range(ROWS):
#                 for j in range(COLS):
#                     self.matrix[i][j] = 0
#
# # Initialize ROS 2
# rclpy.init()
# matrix_publisher = MatrixPublisher()
#
# def draw_matrix(win, matrix):
#     # for i in range(ROWS):
#     #     for j in range(COLS):
#     #         rect = (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
#     #         pygame.draw.rect(win, WHITE if matrix[i][j] == 1 else GREY, rect)
#     #         pygame.draw.rect(win, BLACK, rect, 1)
#     for i in range(ROWS):
#         for j in range(COLS):
#             rect = (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
#             color = WHITE if matrix[i][j] == 2000 else GREY
#             pygame.draw.rect(win, color, rect)
#             pygame.draw.rect(win, BLACK, rect, 1)
#
#             # Render the cell value as text
#             text_surface = font.render(str(matrix[i][j]), True, BLACK)
#             text_rect = text_surface.get_rect(center=(j * CELL_SIZE + CELL_SIZE // 2,
#                                                       i * CELL_SIZE + CELL_SIZE // 2))
#             win.blit(text_surface, text_rect)
#
# # Main loop
# run = True
# while run:
#     rclpy.spin_once(matrix_publisher, timeout_sec=0)
#
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             run = False
#
#     x, y = pygame.mouse.get_pos()
#     col, row = x // CELL_SIZE, y // CELL_SIZE
#     if 0 <= col < COLS and 0 <= row < ROWS:
#         matrix_publisher.update_hovered_cell(row, col, True)
#     else:
#         matrix_publisher.update_hovered_cell(row, col, False)
#
#     matrix_publisher.increment_hovered_cell_value()
#
#
#     win.fill(BLACK)
#     draw_matrix(win, matrix_publisher.matrix)
#     pygame.display.update()
#
# pygame.quit()
# rclpy.shutdown()

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Constants
WIDTH, HEIGHT = 800, 600
COLS, ROWS = 10, 11
CELL_SIZE = min(WIDTH // COLS, HEIGHT // ROWS)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (200, 200, 200)
FONT_SIZE = CELL_SIZE // 2  # Adjust font size as needed

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("11x10 Matrix")
font = pygame.font.SysFont(None, FONT_SIZE)  # Initialize font

# ROS 2 Node and Publisher
class MatrixPublisher(Node):
    def __init__(self):
        super().__init__('my_send_list')
        self.publisher = self.create_publisher(Float32MultiArray, 'heatmap_data_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds
        self.matrix = [[-1 for _ in range(COLS)] for _ in range(ROWS)]

    def timer_callback(self):
        self.publish_list()

    def publish_list(self):
        flat_matrix = [cell for row in self.matrix for cell in row]
        msg = Float32MultiArray(data=flat_matrix)
        self.publisher.publish(msg)
        # Print the matrix data to the terminal
        print(f"Published matrix data: {flat_matrix}")

# Initialize ROS 2
rclpy.init()
matrix_publisher = MatrixPublisher()

def draw_matrix(win, matrix):
    # for i in range(ROWS):
    #     for j in range(COLS):
    #         rect = (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
    #         pygame.draw.rect(win, WHITE if matrix[i][j] == 1 else GREY, rect)
    #         pygame.draw.rect(win, BLACK, rect, 1)
    for i in range(ROWS):
        for j in range(COLS):
            rect = (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            color = WHITE if matrix[i][j] == 2000 else GREY
            pygame.draw.rect(win, color, rect)
            pygame.draw.rect(win, BLACK, rect, 1)

            # Render the cell value as text
            text_surface = font.render(str(matrix[i][j]), True, BLACK)
            text_rect = text_surface.get_rect(center=(j * CELL_SIZE + CELL_SIZE // 2,
                                                      i * CELL_SIZE + CELL_SIZE // 2))
            win.blit(text_surface, text_rect)

# Main loop
run = True
while run:
    rclpy.spin_once(matrix_publisher, timeout_sec=0)  # Allow ROS 2 callbacks

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_SIZE, y // CELL_SIZE
            matrix_publisher.matrix[row][col] = 2000

        if event.type == pygame.MOUSEBUTTONUP:
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_SIZE, y // CELL_SIZE
            matrix_publisher.matrix[row][col] = -1

    win.fill(BLACK)
    draw_matrix(win, matrix_publisher.matrix)
    pygame.display.update()

pygame.quit()
rclpy.shutdown()

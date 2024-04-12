import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Constants
WIDTH, HEIGHT = 800, 600
ROWS, COLS = 11, 10
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
    for i in range(ROWS):
        for j in range(COLS):
            rect = (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            color = WHITE if matrix[i][j] == 255 else GREY
            pygame.draw.rect(win, color, rect)
            pygame.draw.rect(win, BLACK, rect, 1)

            # Render the cell index as text
            cell_index = f"{i},{j}"  # Getting the cell's index
            text_surface = font.render(cell_index, True, BLACK)
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
            matrix_publisher.matrix[row][col] = 255

        if event.type == pygame.MOUSEBUTTONUP:
            x, y = pygame.mouse.get_pos()
            col, row = x // CELL_SIZE, y // CELL_SIZE
            matrix_publisher.matrix[row][col] = -1

    win.fill(BLACK)
    draw_matrix(win, matrix_publisher.matrix)
    pygame.display.update()

pygame.quit()
rclpy.shutdown()

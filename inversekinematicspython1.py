import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Inverse Kinematics Visualization")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Arm parameters
link1_length = 200
link2_length = 150
base_x, base_y = WIDTH // 2, HEIGHT // 2

def draw_text(surface, text, pos, color=BLACK, font_size=24):
    font = pygame.font.Font(None, font_size)
    text_surface = font.render(text, True, color)
    surface.blit(text_surface, pos)

def inverse_kinematics(target_x, target_y):
    dx = target_x - base_x
    dy = target_y - base_y
    distance = math.sqrt(dx ** 2 + dy ** 2)
    distance = min(distance, link1_length + link2_length - 1)  # Ensure the target is reachable

    try:
        cos_angle2 = (dx ** 2 + dy ** 2 - link1_length ** 2 - link2_length ** 2) / (2 * link1_length * link2_length)
        angle2 = math.acos(cos_angle2)
        sin_angle2 = math.sin(angle2)

        k1 = link1_length + link2_length * cos_angle2
        k2 = link2_length * sin_angle2
        angle1 = math.atan2(dy, dx) - math.atan2(k2, k1)
        
        return angle1, angle2
    except ValueError:
        return None, None

def get_joint_positions(angle1, angle2):
    if angle1 is None or angle2 is None:
        return (base_x, base_y), (base_x, base_y)
    joint_x = base_x + link1_length * math.cos(angle1)
    joint_y = base_y + link1_length * math.sin(angle1)
    end_x = joint_x + link2_length * math.cos(angle1 + angle2)
    end_y = joint_y + link2_length * math.sin(angle1 + angle2)
    return (joint_x, joint_y), (end_x, end_y)

def main():
    running = True
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(WHITE)

        # Get mouse position
        target_x, target_y = pygame.mouse.get_pos()

        # Calculate inverse kinematics
        angle1, angle2 = inverse_kinematics(target_x, target_y)

        # Calculate joint positions
        (joint_x, joint_y), (end_x, end_y) = get_joint_positions(angle1, angle2)

        # Draw arm
        pygame.draw.line(screen, RED, (base_x, base_y), (joint_x, joint_y), 5)
        pygame.draw.line(screen, GREEN, (joint_x, joint_y), (end_x, end_y), 5)
        pygame.draw.circle(screen, BLACK, (int(base_x), int(base_y)), 10)
        pygame.draw.circle(screen, BLACK, (int(joint_x), int(joint_y)), 10)
        pygame.draw.circle(screen, BLACK, (int(end_x), int(end_y)), 10)

        # Display mathematical calculations
        draw_text(screen, f"Target: ({target_x}, {target_y})", (10, 10))
        if angle1 is not None and angle2 is not None:
            draw_text(screen, f"Angle1: {math.degrees(angle1):.2f}°", (10, 40))
            draw_text(screen, f"Angle2: {math.degrees(angle2):.2f}°", (10, 70))
            draw_text(screen, f"Joint1: ({int(joint_x)}, {int(joint_y)})", (10, 100))
            draw_text(screen, f"End Effector: ({int(end_x)}, {int(end_y)})", (10, 130))
        else:
            draw_text(screen, "Target out of reach", (10, 40), color=RED)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()

import math
import pygame
import csv
import time
# -----------------
# CSV logging setup
# -----------------
filename=f"sim_output_{int(time.time())}.csv"
file=open(filename, "w", newline="")
writer=csv.writer(file)
writer.writerow([
    "time_s", 
    "x_m", 
    "y_m", 
    "heading_deg", 
    "speed_mmps", 
    "steering_deg", 
    "throttle", 
    "mean_throttle", 
    "brake"
])
# ---------------
# Window settings
# ---------------
WIDTH, HEIGHT=1000, 700
BG_COLOR=(245, 245, 245)
CAR_COLOR=(220, 60, 60)
TRAIL_COLOR=(120, 120, 120)
TEXT_COLOR=(20, 20, 20)
AXIS_COLOR=(210, 210, 210)
STEER_LINE_COLOR=(0, 0, 255)
# -------------------------
# Vehicle / model constants
# -------------------------
FRICTION=0.9296
MIN_THROTTLE=65
MAX_THROTTLE=227
MAX_SPEED_MMPS=13600 # 13.6 m/s in mm/s
LOOP_TIME_S=0.1
THROTTLE_HISTORY_LEN=8
MAX_STEER_DEG=25.0
STEER_RATE_DEG=3.0
BRAKE_DECEL_MMPS=1200 # simple test value per update step
PIXELS_PER_METER=12
# ----------------
# Helper functions
# ----------------
def estimated_speed(mean_throttle:float)->float:
    return max(
        0.0,
        MAX_SPEED_MMPS*(mean_throttle-MIN_THROTTLE)/(MAX_THROTTLE-MIN_THROTTLE),
    )
def clamp(value:float, lo:float, hi:float)->float:
    return max(lo, min(value, hi))
def draw_car(surface, x_px, y_px, heading_deg):
    size=14
    heading_rad=math.radians(heading_deg)
    front=(
        x_px+size*math.sin(heading_rad),
        y_px-size*math.cos(heading_rad),
    )
    left=(
        x_px+size*0.6*math.sin(heading_rad+2.5),
        y_px-size*0.6*math.cos(heading_rad+2.5),
    )
    right=(
        x_px+size*0.6*math.sin(heading_rad-2.5),
        y_px-size*0.6*math.cos(heading_rad-2.5),
    )
    pygame.draw.polygon(surface, CAR_COLOR, [front, left, right])
    # Blue line showing pointing / steering direction
    steer_rad=math.radians(heading_deg+0.8*angle_deg)
    line_end=(
        x_px+28*math.sin(steer_rad),
        y_px-28*math.cos(steer_rad),
    )
    pygame.draw.line(surface, STEER_LINE_COLOR, (x_px, y_px), line_end, 2)
# ----
# Main
# ----
pygame.init()
screen=pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Stage 2 Vehicle Simulator")
clock=pygame.time.Clock()
font=pygame.font.SysFont(None, 28)
# -------------
# Vehicle state
# -------------
x_mm=0.0
y_mm=0.0
heading_deg=0.0
angle_deg=0.0
speed_mmps=0.0
throttle_history=[50]*THROTTLE_HISTORY_LEN
trail=[]
# ---------
# Main loop
# ---------
running=True
while running:
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
    keys=pygame.key.get_pressed()
    # Reset key
    if keys[pygame.K_r]:
        x_mm=0.0
        y_mm=0.0
        heading_deg=0.0
        angle_deg=0.0
        speed_mmps=0.0
        throttle_history=[50]*THROTTLE_HISTORY_LEN
        trail.clear()
    # Controls
    throttle=50
    brake_on=False
    if keys[pygame.K_UP]:
        throttle=150
    if keys[pygame.K_DOWN]:
        brake_on=True
    if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
        angle_deg-=STEER_RATE_DEG
    elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
        angle_deg+=STEER_RATE_DEG
    angle_deg=clamp(angle_deg, -MAX_STEER_DEG, MAX_STEER_DEG)
    # Delayed throttle model
    throttle_history.append(throttle)
    throttle_history.pop(0)
    mean_throttle=sum(throttle_history)/len(throttle_history)
    # Speed model
    est=estimated_speed(mean_throttle)
    momentum=FRICTION*speed_mmps
    speed_mmps=max(momentum, est)
    if brake_on:
        speed_mmps=max(0.0, speed_mmps-BRAKE_DECEL_MMPS)
    # Heading update
    heading_change=angle_deg*0.01 if speed_mmps>0 else 0.0
    heading_deg+=heading_change
    if heading_deg>=360:
        heading_deg-=360
    if heading_deg<0:
        heading_deg+=360
    # Position update
    distance_mm=speed_mmps*LOOP_TIME_S
    heading_rad=math.radians(heading_deg)
    x_mm+=distance_mm*math.sin(heading_rad)
    y_mm+=distance_mm*math.cos(heading_rad)
    # Save trail
    x_px=WIDTH//2+int((x_mm/1000.0)*PIXELS_PER_METER)
    y_px=HEIGHT//2-int((y_mm/1000.0)*PIXELS_PER_METER)
    trail.append((x_px, y_px))
    if len(trail)>500:
        trail.pop(0)
    # Write CSV row
    writer.writerow([
        round(pygame.time.get_ticks()/1000.0, 3),
        round(x_mm/1000.0, 3),
        round(y_mm/1000.0, 3),
        round(heading_deg, 2),
        round(speed_mmps/1000.0, 3),
        round(angle_deg, 2),
        throttle,
        round(mean_throttle, 2),
        int(brake_on)
    ])
    # Draw background
    screen.fill(BG_COLOR)
    # Draw axes
    pygame.draw.line(screen, (210, 210, 210), (WIDTH//2, 0), (WIDTH//2, HEIGHT), 1)
    pygame.draw.line(screen, (210, 210, 210), (0, HEIGHT//2), (WIDTH, HEIGHT//2), 1)
    # Draw trail
    if len(trail)>1:
        pygame.draw.lines(screen, TRAIL_COLOR, False, trail, 2)
    # Draw car
    draw_car(screen, x_px, y_px, heading_deg)
    # Draw text info
    info_lines=[
        f"X (m): {x_mm/1000.0:.2f}",
        f"Y (m): {y_mm/1000.0:.2f}",
        f"Heading (deg): {heading_deg:.1f}",
        f"Speed (m/s): {speed_mmps/1000.0:.2f}",
        f"Speed (km/h): {speed_mmps*3.6/1000.0:.2f}",
        f"Steering angle (deg): {angle_deg:.1f}",
        f"Throttle: {throttle}",
        f"Mean throttle: {mean_throttle:.1f}",
        f"Brake: {int(brake_on)}",
        "Controls: Up=throttle, Down=brake, Left/Right=steer",
    ]
    y_text=20
    for line in info_lines:
        text=font.render(line, True, TEXT_COLOR)
        screen.blit(text, (20, y_text))
        y_text+=28
    pygame.display.flip()
    # Match 100 ms model training
    clock.tick(10)
# -------
# Cleanup
# -------
file.close()
pygame.quit()
# Revised to only have right paddle

import gym
from gym import spaces
import turtle
import numpy as np
import random
import time

class PongEnv(gym.Env):
    metadata = {'render_modes': ['human']}

    def __init__(self, render_mode="human"):
        # print("Environment initialized")
        self.width = 1000
        self.height = 600
        self.hitPaddle = False

        self.observation_space = spaces.Dict({
            'paddle': spaces.Box(-self.height, self.height, shape=(2,)),
            'ball': spaces.Box(-self.width, self.width, shape=(2,))
        })

        self.action_space = spaces.Discrete(3)
        self.sketch = None
        self.setup = True

        self._action_to_direction = {
            0: np.array([0, 20]),
            1: np.array([0, -20]),
            2: np.array([0, 0]),
        }

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode

    def _get_obs(self):
        return {"paddle": self._paddle_location, "ball": self._ball_location}

    def _get_info(self):
        return {"hitPaddle": self.hitPaddle}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        if self.sketch is not None:
            self.sketch.clear()

        self._paddle_location = np.array([400, 0])
        self._ball_location = np.array([0, 0])
        self.hitPaddle = False
        self.hits = 0

        observation = self._get_obs()
        info = self._get_info()

        # Set up the screen
        if self.setup:
            self._setup_render_frame()
            self.setup = False

        # Change velocity of ball every reset
        self.hit_ball.dx = random.uniform(7, 15)
        self.hit_ball.dy = - random.uniform(5, 15)

        # Initialize paddle and ball locations in render frame
        if self.render_mode == "human":
            self._render_frame()

        return observation, info

    def step(self, action):
        # Perform action and update vector
        direction = self._action_to_direction[action]
        self._paddle_location = self._paddle_location + direction

        # Update paddle location in frame (only has to change y since x always remains unchanged for the paddle)
        self.right_pad.sety(self._paddle_location[1])

        # If the ball reaches the right end of the screen or game runtime is more than 30 seconds then terminate
        terminated = False
        if self._ball_location[0] > 500:
            terminated = True

        # Reward is 1 if the ball hits the paddle, 0 otherwise
        reward = 1 if (360 < self._ball_location[0] < 370) and \
                     (self._paddle_location[1] + 40 > self._ball_location[1] > self._paddle_location[1] - 40) else 0

        # Returns the locations of the paddle and the ball (observations of the environment)
        observation = self._get_obs()
        info = self._get_info()

        # If the render mode is human then run the turtle frame
        if self.render_mode == "human":
            self._render_frame()
        return observation, reward, terminated, False, info

    def render(self):
        return self._render_frame()

    def _setup_render_frame(self):
        # Create screen
        self.sc = turtle.Screen()
        self.sc.title("Pong game")
        self.sc.bgcolor("white")
        self.sc.setup(width=self.width, height=self.height)

        # Right paddle
        self.right_pad = turtle.Turtle()
        self.right_pad.speed(0)
        self.right_pad.shape("square")
        self.right_pad.color("black")
        self.right_pad.shapesize(stretch_wid=6, stretch_len=2)
        self.right_pad.penup()
        self.right_pad.goto(self._paddle_location[0], self._paddle_location[1])

        # Ball of circle shape
        self.hit_ball = turtle.Turtle()
        self.hit_ball.speed(40)
        self.hit_ball.shape("circle")
        self.hit_ball.color("blue")
        self.hit_ball.penup()
        self.hit_ball.goto(self._ball_location[0], self._ball_location[1])
        self.hit_ball.dx = random.uniform(7, 15) # 5
        self.hit_ball.dy = - random.uniform(5, 15)  # -5

        # Initialize the score
        self.hits = 0

        # Displays the score
        self.sketch = turtle.Turtle()
        self.sketch.speed(0)
        self.sketch.color("blue")
        self.sketch.penup()
        self.sketch.hideturtle()
        self.sketch.goto(0, 260)
        self.sketch.write(f"Hits: {self.hits}",
                     align="center", font=("Courier", 24, "normal"))

    def _render_frame(self):
        # Updates the screen very frame
        self.sc.update()

        # Sets the new coordinates of the ball based on its velocity
        self.hit_ball.setx(self.hit_ball.xcor() + self.hit_ball.dx)
        self.hit_ball.sety(self.hit_ball.ycor() + self.hit_ball.dy)

        # Sets the new coordinates of the paddle based on the action performed
        self.right_pad.sety(self._paddle_location[1])

        # Update ball location in frame
        self._ball_location = np.array([self.hit_ball.xcor(), self.hit_ball.ycor()])

        # Checking borders
        if self.hit_ball.ycor() > 280:
            self.hit_ball.sety(280)
            self.hit_ball.dy *= -1

        if self.hit_ball.ycor() < -280:
            self.hit_ball.sety(-280)
            self.hit_ball.dy *= -1

        if self.hit_ball.xcor() < -500:
            self.hit_ball.setx(-500)
            self.hit_ball.dx *= -1

        if self.right_pad.ycor() < -230:
            self._paddle_location[1] = -230
            self.right_pad.sety(-230)

        if self.right_pad.ycor() > 230:
            self._paddle_location[1] = 230
            self.right_pad.sety(230)

        # If the ball misses the paddle, then reset the ball
        if self.hit_ball.xcor() > 500:
            self.hit_ball.goto(0, 0)
            self.hit_ball.dy *= -1

        # Paddle ball collision and updating score
        if (self.hit_ball.xcor() > 355 and self.hit_ball.xcor() < 375) and (self.hit_ball.ycor() < self.right_pad.ycor() + 60 and
             self.hit_ball.ycor() > self.right_pad.ycor() - 60):
            self.hitPaddle = True
            self.sketch.clear()
            self.hit_ball.setx(355)
            self.hit_ball.dx *= -1
            self.hits += 1

        # Update the scoreboard
        self.sketch.write(f"Hits: {self.hits}",
                          align="center", font=("Courier", 24, "normal"))


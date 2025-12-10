from Classes.Ball import Ball
from Classes.Mobius_Stair import Mobius_stair
from utils.maya_helpers import clear_scene


def main():
    clear_scene()

    my_stair = Mobius_stair()
    my_stair.make_strip()
    my_stair.get_face_center()
    my_stair.get_face_normal()

    ball = Ball(1, "BouncingBall")
    ball.ball_rig()
    ball.bounce_path(my_stair)
    # ball.line_on_path()
    ball.bounce(my_stair)


if __name__ == "__main__":
    main()

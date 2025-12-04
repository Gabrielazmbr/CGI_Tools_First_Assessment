from Classes.Ball import Ball
from Classes.Mobius_Stair import Mobius_stair
from utils.maya_helpers import clear_scene


def main():
    clear_scene()

    my_stair = Mobius_stair()
    my_stair.make_strip()
    my_stair.get_face_center()
    my_stair.get_face_normal()

    bolita = Ball(1, "BouncingBall")
    bolita.ball_rig()
    bolita.bounce_path(my_stair)
    bolita.line_on_path()
    bolita.bounce(my_stair)


if __name__ == "__main__":
    main()

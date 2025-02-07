import pygame
import numpy as np
import time


class Joystick:
    def __init__(self, joystick_id):
        pygame.init()
        self.joystick_id = joystick_id

    def joystick_initialization(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(self.joystick_id)
            self.joystick.init()
            print(f"Detected joystick: {self.joystick.get_name()}")
        else:
            print("No joystick detected.")
            raise ValueError("No joystick detected.")

    def _get_axis(self):
        axis_ls_horiz = self.joystick.get_axis(0)
        axis_ls_vert = self.joystick.get_axis(1)

        axis_rs_horiz = self.joystick.get_axis(3)
        axis_rs_vert = self.joystick.get_axis(4)

        axis_lb = self.joystick.get_axis(2)
        axis_rb = self.joystick.get_axis(5)

        return np.array([axis_ls_horiz, axis_ls_vert, axis_lb, axis_rs_horiz, axis_rs_vert, axis_rb])

    def _get_button(self):
        bu_a = self.joystick.get_button(0)
        bu_b = self.joystick.get_button(1)
        bu_x = self.joystick.get_button(2)
        bu_y = self.joystick.get_button(3)
        bu_lt = self.joystick.get_button(4)
        bu_rt = self.joystick.get_button(5)
        bu_start = self.joystick.get_button(6)
        bu_reset = self.joystick.get_button(7)

        return np.array([bu_a, bu_b, bu_x, bu_y, bu_lt, bu_rt, bu_start, bu_reset])

# [axis_ls_horiz, axis_ls_vert, axis_lb, axis_rs_horiz, axis_rs_vert, axis_rb, \
# bu_a, bu_b, bu_x, bu_y, bu_lt, bu_rt, bu_start, bu_reset]
    def get_joystick(self):
        pygame.event.pump()
        joystick_axis_sts = self._get_axis()
        joystick_button_sts = self._get_button()

        return np.concatenate([joystick_axis_sts, joystick_button_sts])

    def print_info(self):
        if pygame.joystick.get_count() > 0:
            for i in range(pygame.joystick.get_count()):
                print(
                    f"joystick name {i}: {pygame.joystick.Joystick(i).get_name()}")
        else:
            print("No joystick found")


def test_real_teleop(joystick):

    # joystick.print_info()

    while True:
        # print("joystick:", joystick.get_joystick())
        print("axis_ls_horiz:", joystick.get_joystick()[0])
        print("axis_ls_vert:", joystick.get_joystick()[1])
        print("axis_lb:", joystick.get_joystick()[2])
        print("axis_rs_horiz:", joystick.get_joystick()[3])
        print("axis_rs_vert:", joystick.get_joystick()[4])
        print("axis_rb:", joystick.get_joystick()[5])
        print("button_a:", joystick.get_joystick()[6])
        print("button_b:", joystick.get_joystick()[7])
        print("button_x:", joystick.get_joystick()[8])
        print("button_y:", joystick.get_joystick()[9])

        print("button_lt:", joystick.get_joystick()[10])
        print("button_rt:", joystick.get_joystick()[11])
        print("button_start:", joystick.get_joystick()[12])
        print("button_reset:", joystick.get_joystick()[13])

        time.sleep(1)


def print_joystick_status(joystick):
    while True:
        joystick_data = joystick.get_joystick()
        joy_axis_neg_indices = np.where(joystick_data < -0.9)[0]
        joy_sts_indices = np.where(joystick_data > 0.9)[0]

        for value in joy_axis_neg_indices:
            if value == 0:
                print("LS Left")
            elif value == 1:
                print("LS Up")
            elif value == 2:
                continue
            elif value == 3:
                print("RS Left")
            elif value == 4:
                print("RS Up")
            elif value == 5:
                continue

        for value in joy_sts_indices:
            if value == 0:
                print("LS Right")
            elif value == 1:
                print("LS Down")
            elif value == 2:
                print("LB Pressed")
            elif value == 3:
                print("RS Right")
            elif value == 4:
                print("RS Down")
            elif value == 5:
                print("RB Pressed")
            elif value == 6:
                print("A Pressed")
            elif value == 7:
                print("B Pressed")
            elif value == 8:
                print("X Pressed")
            elif value == 9:
                print("Y Pressed")
            elif value == 10:
                print("LT Pressed")
            elif value == 11:
                print("RT Pressed")
            elif value == 12:
                print("Start Pressed")
            elif value == 13:
                print("Reset Pressed")
        time.sleep(0.1)


if __name__ == '__main__':
    joystick = Joystick(0)
    joystick.joystick_initialization()
    # test_real_teleop(joystick)
    print_joystick_status(joystick)

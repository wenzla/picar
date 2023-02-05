import car
import time

car.startup()
time.sleep(0.5)

sweep = car.us_sweep()
car.right_turn()
car.forward_step(7)
time.sleep(3)
car.left_turn()
car.forward_step(8)
car.left_turn()
car.forward_step(7)
car.right_turn()
car.forward_step(6)
car.stop_car()
car.left_turn()
car.stop_car()

sweep = car.us_sweep()

car.left_turn()
car.forward_step(5)
car.right_turn()
car.forward_step(11)
car.stop_car()

#



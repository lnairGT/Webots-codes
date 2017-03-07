# Webots-codes

Contains Webots simulator code for behavioral controllers. 

behavioral_controller.c - Uses action selection behavior coordination mechanism to select one of four simple behaviors: MOVE_TO_FOOD, 
FIGHT_CONSPECIFIC, RETREAT and AVOID_OBSTACLE.

e-puck_attach_new.c - Simple code to travel to a specified target location using GPS and Compass

e_puck_comfort.c - Attachment behavior implementation as described in Likhachev, Maxim, and Ronald C. Arkin. "Robotic comfort zones." Intelligent Systems and Smart Manufacturing. International Society for Optics and Photonics, 2000.
Combined with MOVE_TO_FOOD behavior using Action Selection coordination strategy

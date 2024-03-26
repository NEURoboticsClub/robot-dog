"""@package docstring
Documentation for this module

More details
"""

import asyncio

# for bridge nodes sockets
import socket

from motor_controller import MotorController

# Get local machine name
SERVER_HOST = socket.gethostname()
CPU_SUB_SERVER_PORT = 9999
MC_SUB_SERVER_PORT = 9998

async def close_key(m):
	await m.close_moteus()
	m.mprint("Moteus Closed Properly")

async def main(controller: MotorController):
	"""
<<<<<<< HEAD
		TODO: add details
		Sockets allow you to establish network connections over various network protocools 
		(this project uses simple IPv4) to send and recieve data.  
		Loops until keyboard interrupt
=======
		Loops until keyboards interrupt
>>>>>>> 906f694e96801072584e70a4f82181c465a744df
		Args:
            controller: MoteusController
        Returns:
            asyncio.gather

        Raises:
            KeyboardInterrupt
	"""
	# to = 3                      #0.1 seems to be the lower limit for a standalone motor. This is max torque.
	# vel = 1

	# board can sense where position 0 is via absolute encoder within 1/10 rotation this offset changes where it's zero
	# is

	# sockets:
	# 1a. init the socket and set the timeout to 10 seconds. purpose: listen to cpu_sub node
	cpu_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	cpu_sub_socket.settimeout(10.0)
	# 1b. connect to remote socket on port 9999 on SERVER_HOST
	cpu_sub_socket.connect((SERVER_HOST, CPU_SUB_SERVER_PORT))

	# 2a. init the socket and set the timeout to 10 seconds. purpose: send msg to mc_sub node
	mc_sub_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	mc_sub_socket.settimeout(10.0)
	# 2b. connect to remote socket on port 9998 on SERVER_HOST
	mc_sub_socket.connect((SERVER_HOST, MC_SUB_SERVER_PORT))

	# 3. create three coroutines that'll run concurrently.
	# 3a. controller_task will  
	controller_task = asyncio.create_task(controller.run())
<<<<<<< HEAD
	# 3b. cpu_task will set attributes to the 12 motor controllers
	cpu_task = asyncio.create_task(get_cpu_command(cpu_sub_socket, controller))
	# 3c. mc_task will 
	mc_task = asyncio.create_task(send_mc_states(controller, mc_sub_socket))
=======
	cpu_task = asyncio.create_task(m.get_cpu_command(cpu_sub_socket, controller))
	mc_task = asyncio.create_task(m.send_mc_states(controller, mc_sub_socket))
>>>>>>> 906f694e96801072584e70a4f82181c465a744df

	# 4. schedule the coroutines to be run. stop running on keyboard interrupt (any keyboard input).
	try:
		await asyncio.gather(controller_task, cpu_task, mc_task)
	except KeyboardInterrupt:
		await close_key(controller)
		return

	controller.mprint(controller.get_parsed_results())


if __name__ == '__main__':
	loop = asyncio.get_event_loop()
	m = loop.run_until_complete(MotorController.create(ids=[[2], [], [], [], []]))
	try:
		loop.run_until_complete(main(m))
	except KeyboardInterrupt:
		loop.run_until_complete(close_key(m))

# to add:
# flux braking- moteus defaults to discharging voltage when braking to DC power bus
# servo.flux_brake_min_voltage and servo.flux_brake_resistance_ohm can change this

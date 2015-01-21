LIBS=wpi
TEAM=1786
SSH_OPTIONS=-q -i id_rsa -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no

all: deploy

deploy: build
	@echo "Copying FRCUserProgram"
	@ssh $(SSH_OPTIONS) lvuser@roborio-1786.local 'rm /home/lvuser/FRCUserProgram'
	@scp $(SSH_OPTIONS) -o "LogLevel QUIET" FRCUserProgram lvuser@roborio-1786.local:/home/lvuser/FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'

build:
	@echo "Building FRCUserProgram"
	arm-frc-linux-gnueabi-g++ robot.cpp -o FRCUserProgram -l$(LIBS) -std=c++11

clean:
	rm FRCUserProgram

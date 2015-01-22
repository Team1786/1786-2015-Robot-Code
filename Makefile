LIBS=wpi
override CFLAGS +=-l$(LIBS) -std=c++11
TEAM=1786
SSH_OPTIONS=-q -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no
SSH_SSHPASS=$(shell command -v sshpass >/dev/null 2>&1 && echo -n "sshpass -p ''")

all: deploy

build: FRCUserProgram

FRCUserProgram: robot.cpp
	@echo "Building FRCUserProgram"
	arm-frc-linux-gnueabi-g++ robot.cpp -o FRCUserProgram $(CFLAGS)

clean:
	rm FRCUserProgram

deploy: build
	@echo "Copying FRCUserProgram"
	@ssh $(SSH_OPTIONS) lvuser@roborio-1786.local 'rm -f /home/lvuser/FRCUserProgram'
	@scp $(SSH_OPTIONS) -o "LogLevel QUIET" FRCUserProgram lvuser@roborio-1786.local:/home/lvuser/FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'

restart: FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'

stop:
	@echo "Restarting FRCUserProgram"
	@$(SSH_SSHPASS) ssh $(SSH_OPTIONS) admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t'

LIBS=wpi
TEAM=1786

all: deploy

deploy: build
	@echo -e "Copying FRCUserProgram"
	@ssh -q lvuser@roborio-1786.local 'rm /home/lvuser/FRCUserProgram'
	@scp -o "LogLevel QUIET"  FRCUserProgram lvuser@roborio-1786.local:/home/lvuser/FRCUserProgram
	@echo "Restarting FRCUserProgram"
	@ssh -qi id_rsa admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'
# delete next line when previous works
	@ssh -qi id_rsa admin@roborio-1786.local '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcRunRobot.sh'

build:
	@echo "Building FRCUserProgram"
	arm-frc-linux-gnueabi-g++ robot.cpp -o FRCUserProgram -l$(LIBS) -std=c++11

clean:
	rm FRCUserProgram

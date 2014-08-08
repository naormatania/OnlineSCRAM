import re
import sys
import math
import os

support_collision = True

GOAL_PATTERN = "\(\d+,\d+\)"
ALGO_PATTERN = "/(\w+)"
START_TIME_PATTERN = "Time (\d+)"
LOCATIONS_PATTERN = "(\d+)-\(([-]*\d+),([-]*\d+)\)"
ASSIGNMENT_PATTERN = "(\d+)-(\d+)"
FINISH_TIME_PATTERN = "Robot (\d+) finished At time (\d+)"
ABORT_TIME_PATTERN = "Assignment aborted At time (\d+)"
NUM_COLLISIONS_PATTERN = "number of collisions is (\d+)"

NUM_OF_ALGORITHMS = 4
'''
\item \textbf{Makespan}: 
\item \textbf{DistSum}: 
\item \textbf{AveArrTime}: The average arrival time over all tasks.
'''

def sumDist(pairLocations):
	sumD = 0
	locA, locB = pairLocations
	for robotIndex in locA:
		sumD = sumD + math.sqrt((locB[robotIndex][0]-locA[robotIndex][0])**2+\
			(locB[robotIndex][1]-locA[robotIndex][1])**2)
	return sumD

if __name__ == "__main__":
	logFilePath = sys.argv[1]
	logFile = open(logFilePath, "rb")
	goals = map(eval, re.findall(GOAL_PATTERN,logFile.readline()))
	print "number of goals is %d" % len(goals)
	for num_goal in xrange(NUM_OF_ALGORITHMS):
		algo = re.findall(ALGO_PATTERN,logFile.readline())[0]
		print "assignment algorithm is %s" % algo
		lines = []
		pattern = re.compile(ALGO_PATTERN)
		while True:
			line = logFile.readline()
			if pattern.match(line) or (line == ''):
				logFile.seek(-1*len(line), os.SEEK_CUR)
				break
			else:
				lines.append(line)
		if support_collision:
			colllsions_num = int(re.findall(NUM_COLLISIONS_PATTERN, lines[-1])[0])
			print "number of collisons: %d" % colllsions_num
		lines = lines[:-1]
		Makespan = 0
		robotLocationsList = []
		arrivalTimes = []
		ongoing_targets = {}
		end_time = 0
		for i in xrange(0,len(lines),5):
			start_time = int(re.findall(START_TIME_PATTERN,lines[i])[0])
			for key in ongoing_targets.keys():
				ongoing_targets[key] = ongoing_targets[key] + start_time - end_time
			# print "start time is %d" % start_time
			robotsLocations = dict(map(lambda x: (int(x[0]), (int(x[1]),int(x[2]))),\
				re.findall(LOCATIONS_PATTERN,lines[i+1])))
			#print "robots locations are %s" % str(robotsLocations)
			targetsLocations = dict(map(lambda x: (int(x[0]), (int(x[1]),int(x[2]))),\
				re.findall(LOCATIONS_PATTERN,lines[i+2])))
			# we assume no two targets of the location can be on the same time
			for _, target in targetsLocations.iteritems():
				if not ongoing_targets.has_key(target):
					ongoing_targets[target] = start_time
			#print "targets locations are %s" % str(targetsLocations)
			tasksAssignments = dict(map(lambda x: (int(x[0]), int(x[1])),\
				re.findall(ASSIGNMENT_PATTERN,lines[i+3])))
			if (re.findall(FINISH_TIME_PATTERN,lines[i+4]) != []):
				finished_robot, end_time = map(int, re.findall(FINISH_TIME_PATTERN,lines[i+4])[0])
				#arrivalTimes.append(Makespan+end_time-start_time)
				finished_task = tasksAssignments[finished_robot]
				arrivalTimes.append(end_time-ongoing_targets.pop(targetsLocations[finished_task]))
			else:
				end_time = int(re.findall(ABORT_TIME_PATTERN,lines[i+4])[0])
			#print "finished robot is %d" % finished_robot
			#print "end time is %d" % end_time
			if (i == len(lines)-9):
				for j in xrange(4):
					finished_robot, end_time = map(int, re.findall(FINISH_TIME_PATTERN, \
						lines[i+5+j])[0])
					finished_task = tasksAssignments[finished_robot]
					try:
						arrivalTimes.append(end_time-ongoing_targets.pop(targetsLocations[finished_task]))
					except:
						pass
					#arrivalTimes.append(Makespan+end_time-start_time)
			robotLocationsList.append(robotsLocations)
			Makespan = Makespan + end_time - start_time
			if (i == len(lines)-9):
				break

		robotLocationsList.append(targetsLocations)
		locationsPairs = zip(robotLocationsList[0:len(robotLocationsList)-1],\
			robotLocationsList[1:len(robotLocationsList)])
		DistSum = sum(map(sumDist, locationsPairs))
		AveArrTime = 1.0*sum(arrivalTimes)/len(arrivalTimes)
		print "Makespan (The total completion time): %d" % Makespan
		print "DistSum (The sum of paths over all robots): %d" % DistSum
		print "AveArrTime (The average arrival time over all tasks): %f" % AveArrTime

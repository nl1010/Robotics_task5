#!/usr/bin/python
import ku_experiment

if __name__ == "__main__":  
  for i in range(0, 1000, 100):
    print "Testing Motor 0 with KU = " + str(i)
    ku_experiment.main(i)

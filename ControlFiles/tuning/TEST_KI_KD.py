#!/usr/bin/python
import kd_ki_experiment
import ku_experiment 

FOUND_KU = 700
FOUND_PU = 0.7# 0.25

if __name__ == "__main__":  
 #for i in range(80, 120, 5):
 #  for j in range(80, 120, 5):
 #    print "Testing Motor 0 with Ki mult = " + str(i * 0.01) + " and Kd mult = " + str(j * 0.01)
 #    kd_ki_experiment.main(ku=FOUND_KU, pu=FOUND_PU, percentage_modifier_ki=i*0.01, percentage_modifier_kd=j*0.01, distance=20)
  #for i in range(FOUND_KU - 50, FOUND_KU + 50, 10):
  #  kd_ki_experiment.main(ku=FOUND_KU, pu=FOUND_PU, percentage_modifier_kd=1, percentage_modifier_ki=0.5, distance=20)
  for i in range(6, 10, 1):
    kd_ki_experiment.main(ku=FOUND_KU, pu=(i * 0.1), percentage_modifier_kd=1, percentage_modifier_ki=0.5, distance=20)



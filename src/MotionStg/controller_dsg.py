import gclib

g = gclib.py()
g.GOpen('169.254.93.220 --direct')
print(g.GInfo())


    #Motion Complete
print('Motion Complete')
c = g.GCommand #alias the command callable
c('AB') #abort motion and program
c('MO') #turn off all motors
c('SHA') #servo A
c('SHB') #servo A
c('SPA=100000') #speead, 1000 cts/sec
c('PRA=-300000') #relative move, 3000 cts
c('SPB=100000') #speead, 1000 cts/sec
c('PRB=-300000') #relative move, 3000 cts
print(' Starting move...')
c('BGA') #begin motion
c('BGB') #begin motion
g.GMotionComplete('A')
g.GMotionComplete('B')
print(' done.')
del c #delete the alias

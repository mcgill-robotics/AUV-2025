import smach

class Test:
     def __init__(self):
          global sm
          self.something = sm

     def change(self):
          self.something = 2


sm = smach.StateMachine(outcomes=['success', 'failure']) 
temp = Test()
temp.change()
print(temp.something)
print(sm)
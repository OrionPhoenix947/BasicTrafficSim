import matplotlib.pyplot as plt
import numpy as np
import random
from alive_progress import alive_bar

#Global variables
#axis_bounds = 10
Max_time_step = 10000
probability_to_decel = 0.1
Max_test_number = 1
Length_of_lane1 = 1000

################

#Functions 
class lane():
    """
    Lane class contains functions related to a basic simulation of traffic.
    A single lane with discrete positions, 'cars' within the lane also have discrete velocity.
    On the lane there is a defined max_speed that cars will not exceed.


    """
    def __init__(self, lane_steps:int, max_speed:int):
        """
        To initialise the lane it needs to know its length and max speed

        Input
        ---------------------------------
        lane_steps: int
        Defines the length of the lane in discrete steps and corresponds to the array length

        max_speed: int
        Defines the maximum speed that the cars may travel

        
        Assignments to Class object lane
        ---------------------------------
        self.lane:nd.array(int)
        Creates an array assigned to class object lane initally filled with -1's in order to indicate positions that cars are not in

        self.length:int
        Assigns access to the lanes length

        self.max_speed:int
        Assigns access to the max_speed

        self.car_pos:array[int]
        Defines an empty array to be the indices where cars are.
        Allowing to quickly compare car positions without searching the lane each time
        """
        self.lane = np.full(lane_steps,-1, int) #Sets up a 1D array with a length of lane_steps
        self.length = lane_steps
        self.max_speed = max_speed 
        self.car_pos = []
        pass
    def initialise_manual_cars(self,lanearray:np.ndarray, cararray:np.ndarray):
        """
        Allows for manual implementation of a lane array and cars, mainly for debugging

        If this remains know that It should have been removed as its use was only for debug
        """
        self.lane = lanearray
        self.car_pos = cararray
    def initialise_random_cars(self,num_of_cars:int):
        """
        Function to setup the inital location of cars based on random positions

        It does this by setting up a random array with no duplicates to assign random starting positions to cars

        Assumption - Cars will start with 0 velocity

        Input
        ---------------------------
        self - lane class
        (For a specific lane called 'laneobj' the use of any function 'func' that uses self is laneobj.func)

        num_of_cars:int
        Number of cars that should be assigned to the array 
        Note: this should be less than the length of the lane and I should do some basic value checking

        Process
        ---------------------------
        self.car_num:int
        Assigns the number of cars to the lane 

        Updates self.car_pos
        Includes a random sample of unique indices within the lane that cars will start at

        Updates self.lane
        Where there is a car it assigns the value 0 to the lane. Corresponding to the starting speed of the car 
        """

        self.car_num = num_of_cars

        car_indices = random.sample(range(self.length), num_of_cars) 
        #Random array of indices to determine position of cars

        self.car_pos = np.sort(np.array(car_indices)) #Updates the array of car indices to class
        #Ive sorted it in hopes to maybe reduce  interations in for loops in the future
        
        #I will assume cars will start with no speed
        for index in car_indices:
            self.lane[index] = 0
            #Sets the speed to 0 at the position of the car in the lane array

    def update_car_kinetics_circularly(self):
        """
        Updates cars position and velocity within a lane based on a few simple rules
        Note: the speed and position are discrete integers

        - For a given car, if there isnt a car within its velocity step + 1 and so long as its velocity is less than the speed limit
        the car will accelerate by adding 1 to its velocity

        - For a given car, if there is a car within its velocity step it will decelerate to 1 less than the distance between the cars

        #Currently not implemented
        # For a given car, if the velocity is not 0 there is a probability p that the speed decreases by 1
        #Currently not implemented

        - A car will move by its velocity each timestep
        """

        ###Acceleration and decceleration

        for car_index in self.car_pos:
            #Iterates over all the cars

            cant_accelerate = False
            #Allows for the potential to accelerate 

            positional_index = np.argmax(self.car_pos== car_index)
            #Defines the car's position in the car index array

            for next_cars in range(self.max_speed):
                #Iterates over car_positions from car_index's postion to car_index position + max_speed to reduce unneccesary checking
                ##This only works as we sort the array of car positions

                car2_index = self.car_pos[(positional_index +1+ next_cars)%self.car_num]
                #Defines car two index based 

                #Now that I use a sorted form I could break once I find that the next_car isnt within velocity range

                if car_index < car2_index <= car_index + self.lane[car_index] + 1: 
                    #Checks if there is a car infront that it would hit
                    ##Will also check velocity + 1, which will leave this unchanged but will not allow 

                    self.lane[car_index] = (car2_index - car_index) - 1
                    #Reduces speed of the car by to the distance to the 'next' car - 1
                    #In order to avoid collision or overlap

                    if car2_index == car_index + 1:
                        #Breaks the for loop
                        break   
                    else:
                        #This covers the case where the loop should break before completing (indicating it will collide so the car wont accelerate)
                        cant_accelerate = True

                elif self.length -1 - self.lane[car_index]<= car_index and not cant_accelerate:
                    #Checks to see if the indexer is close to the end of the array
                    #if so need to check start of the array
                    #If break_before_end_is true there is already a collision before this transition
                    #(The else if wont catch i think if it occured in a prior iteration)

                    if car2_index <= self.lane[car_index]- (self.length-1 - car_index):
                        #If the index of a car is early enough for a collision into
                        # Car 2 positon < Speed of Car 1 - (Length of track - Car 1 position)
                        self.lane[car_index] = (car2_index + (self.length-1-car_index))
                        #Sets the velocity of the car 
                        cant_accelerate = True

                if next_cars == (self.max_speed - 1) and cant_accelerate:
                    #Guarantess the else below will be executed by breaking the for loop before it ends normally
                    #Allowing the car to accelerate
                    break
            else:
                #No cars will be hit so the car can accelerate
                if self.lane[car_index] < self.max_speed:
                    #Checks to see if the velocity of the car is less than the speed limit
                    self.lane[car_index] += 1  
                    #Increases the speed by 1
        
        ###Car motion and randomisation

        #Instead of updating motion in the same loop which would cause issues when comparing I will setup a new loop
        ## perhaps this can be improved later
        for car_index1 in self.car_pos:

            ### Randomisation
            if self.lane[car_index1] != 0:
                if random.random() < probability_to_decel:
                    self.lane[car_index1] -= 1
            
            ###Car motion

            car_speed1 = self.lane[car_index1] 
            #Finds the car's current speed

            self.lane[car_index1] = -1 
            #Removes the car either temporarily (if v = 0) or allows it to be freely moved

            self.lane[(car_index1 + car_speed1) % self.length] = car_speed1 
            #Sets the car speed to the new position, requires modulo as we assume a circular road

            #Causes issues if function is called outside, as counters is not defined globally
            for counter in counters: #I dont like this, should think of alternative
                counter.update_counter(car_index1, car_speed1)

            #This is probably slower than having an iterator but idk for sure
            self.car_pos[np.argmax(self.car_pos == car_index1)] = (car_index1 + car_speed1) % self.length
            #Renames the old index to point to the new index
            
        self.car_pos = np.sort(self.car_pos) #Sorts car positions at the end in order for the accel check to be efficient and correct

class Car_counter():
    def __init__(self, counter_location):
        self.pos = counter_location
        self.count = 0
        self.vel_count = 0
        pass
    def reset_counter(self):
        self.count = 0
        self.vel_count = 0
    def update_counter(self,car_position:int, car_velocity:int):
        if car_position <= self.pos < car_position + car_velocity: 
            #can this double count and is that ok>
            
            self.count += 1

            self.vel_count += car_velocity
            #Experimenting with a different flow counter

def check_for_multicar(lane:lane):
    for i in range(len(lane.car_pos)-1):
            if lane.car_pos[i] == lane.car_pos[i+1]:
                print("Raise error")
                raise RuntimeError("There are positions with multiple cars")
            
def format_lane_array(array:np.ndarray[int]):
    """
    Formats the lane array to be printed in the terminal

    Input 
    -------------------------
    array:ndarray(int)

    Process
    -------------------------
    Takes the input integer array and makes a string

    Instances where -1 is used it is replaced by '_' indicating an empty position
    Where the is a car it instead adds the corresponding number in that position

    Returns
    --------------------------
    lane_array:str
    String corresponding to the formatted array
    """
    lane_array:str = ""
    for step in array:
        if step == -1:
            lane_array += "_"
        else:
            lane_array += str(step)
    return lane_array
    
def lane_simulation(Length_of_lane:int, Speed_limit:int,Number_of_cars:int,Max_time_step:int, print_lane:bool):
    """
    Defines how a lane is created and updated
    """
    lane1 = lane(Length_of_lane,Speed_limit)
    #Creates a lane object
    lane1.initialise_random_cars(Number_of_cars)
    #Sets up cars at random positions along the lane
    print(f"Car number: {Number_of_cars}")
    for t in range(Max_time_step):
        #Time step for loop
        if print_lane:
            print(f"t = {t} >>> | {format_lane_array(lane1.lane)} |")
        #Prints a formatted lane at a given time step

        check_for_multicar(lane1)
        #Error checking for multiple cars occupying one spot (For debugging purposes to make sure code is consistent)

        lane1.update_car_kinetics_circularly()
        #Updates cars based on the basic rules defined in the lane class



################
#This is outside main in order for cProfile to work
counters:list[Car_counter] = []

counter_locations = [Length_of_lane1 - 1] 
for counter_index in range(len(counter_locations)):
    counters.append(Car_counter(counter_locations[counter_index]))



if __name__ == '__main__':
    #Local variables
    #Number_of_cars = 25
    
    Speed_limit = 5
    print_lane = False
    
    #lane1 = lane(Length_of_lane1,Speed_limit)
    #lane1.initialise_random_cars(Number_of_cars)
    data_array = [[],[],[]] # Density array, Counter value array # Time average velocity passing point
    #Sets up an array for the counter class

        #Adds a counter at its position to the counters array 
    for test_num in range(1,Max_test_number + 1):
        print(f"Test number is {test_num}")
        with alive_bar(Length_of_lane1, title='Processing Number of cars', length=80, bar='notes') as bar: #, receipt = True
            for num_of_car in range(Length_of_lane1):   
                lane_simulation(Length_of_lane1,Speed_limit,num_of_car,Max_time_step,print_lane)
                bar()
                for counter_index in range(len(counter_locations)):
                    if test_num == 1: # This can likely be unraveled
                        data_array[0].append(num_of_car/Length_of_lane1)
                    #Adds the absolute density to an array
                    data_array[test_num].append(counters[counter_index].count/ Max_time_step)
                    #Adds time averaged number of cars passing a point to the data array
                    #data_array[2].append(counters[counter_index].vel_count/ Max_time_step)
                    #Adds time averaged velocity of passing cars to the data array
                    counters[counter_index].reset_counter()
                    #Resets both counters to avoid data leaking to the next density step

    #lane_simulation(Length_of_lane1,Speed_limit,20, Max_time_step, print_lane)

    fig, ax = plt.subplots()

    ax.set( xlabel = 'Absolute Car density', ylabel = 'Time averaged Flow of cars')
    for test_num in range(1,Max_test_number + 1):
        ax.plot(data_array[0], data_array[test_num])

    plt.show()

    #print(data_array[0])
    #print("66666666666666666666666666666666666666666666 ")
    #print(data_array[1])

    fig, ax = plt.subplots()

    ax.set( xlabel = 'Car density', ylabel = 'Time averaged Velocity of cars')
    ax.plot(data_array[0], data_array[2])

    plt.show()

    #print(data_array[1])
    #print("66666666666666666666666666666666666666666666 ")
    #print(data_array[2])

#with alive_bar(n, title='Processing', length=20, bar='notes') as bar:
#bar()

#fig, ax = plt.subplots()


#ax.set(xlim=[-axis_bounds, axis_bounds], ylim=[-axis_bounds, axis_bounds], xlabel='x', ylabel='y')
#ax.legend()

#plt.show()

#

#Assumption, that the order of accel, slow,rand, move is the right one
#Assumption that cars are initially have velo =0 

#Thought potentially that dont have to check 2nd cars position

#There is an issue of assuming cars will never overtake.

#Numerical stability concerns - so far as I work with integers and thus do not suffer the same rounding errors as floats I believe I
#am fine. Within the exception of stability of certain sorting and numpy array methods that may break for large integers.


#Check if loops have to be different variables to neaten up the code or not

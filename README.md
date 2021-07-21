# Robotic Practical Assignment

The Robot Operating System (ROS) is a set of software libraries for building robot
applications.

Use ROS2 (a newer version of ROS) and write a program that communicates with 
a database (MySQL, PostgreSQL, etc.).

### Assignment:

1. Create a database that has two rows, the first row holds robot linear velocity,
and the second row holds robot angular velocity.

**Solution:**

I used SQLite database. The table I created was named **velocitylog** and it was of the
following structure:

	CREATE TABLE "velocitylog" (
		"id"	INTEGER NOT NULL,
		"linear"	REAL NOT NULL,
		"angular"	REAL NOT NULL,
		"timestamp"	TEXT NOT NULL,
		PRIMARY KEY("id")
	)
	
The database is located in the directory called **data** and it is called robot.sqlite3.

	
2. Write programs (nodes)
	
	1) Program reads user input from terminal and writes the data to the database.
	
	**Solution:**
	
	In order to solve this problem I created a service called
	**velocity_logging_service.py** and a client **velocity_posting_cliend.py**.
	Those two services are located in **src/hello_robot** package. In addition
	I had to create a supporting package **src/robot_interfaces**, which contains
	and interface used by the service.
	
	2) Program executing commands saved to the database. The program has to be
	called every 5 seconds.
	
	**Solution:**
	
	The node addressing this specifical problem is called **command_publisher**
	
	
3. Modify the program so that a user could use **/stop** parameter 
of the **std_msgs/msg/Bool** format, and if the parameter value is set to true
all the records from the database are deleted.


**Solution:**

I tried the solution with parameters, however I found it clumsy, and I ended up
adding a command line optional parameter to **command_publisher**

If you run the publisher using reset option, all the commands saved in **processing**
table will be deleted:

	ros2 run hello_robot command_publisher --reset
	
### Comments:

1. I created two tables in the database, the first table is called **velocitylog**
and all the commands are saved to this table. In parallel I created a table called
**processing**, and the commands broadcasted by **command_publisher** are taken from 
this particular table.


### Usage:

1. Create a ROS2 workspace

2. Copy **data/** and **src/** directories to the workspace.

3. Build the project

	colcon build
	
4. Activate project workspace

```
    source install/setup.bash
```
	
5. First let's add some data to our database

```
    #Start velocity log service
    ros2 run hello_robot velocity_log_service
```
	
After executing this command velocity_log_service is ready to accept incoming commands.

In order to add data to the database we need to use **velocity_log_client**

```
    #Entering the data to database
    ros2 run hello_robot velocity_log_client 1 1
```
	
The command above creates a record in the **velocitylog** table and in
the **processing** table. The transmitted data is echoed by velocity_log_service.


6. Next we want to pass the commands to **turtlesim** and hopefully turtle starts moving
following our commands.

```
    #Start turtlesim
    ros2 run turtlesim turtlesim_node
```

Open new terminal:

```
    #Pass commands from db to turtle
    ros2 run command_publisher
```
	
	

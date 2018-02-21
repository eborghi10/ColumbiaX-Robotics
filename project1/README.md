# CSMM.103x | Project 1

In this assignment you are tasked with writing a **node that subscribes to a topic and publishes into another one**. Your code will subscribe to a topic called _'two_ints'_, on which a custom message containing two integers can be broadcast. Make sure to familiarize yourself with the message format of this topic (have a look at the TwoInts.msg in the msg directory). Those two integers are to be added and the result published to topic 'sum' as an Int16. 

## Testing

You will need to run 3 terminal commands:

1) Execute the ROS node that creates 2 topics. One that subscribes to `/two_ints` and the other that publishes into `/sum`.

- `roslaunch project1 execute.launch`

2) Subscribe to the topic `/sum`.

- `rostopic echo /sum`

3) Publish into the topic `/two_ints`.

- `rostopic pub /two_ints project1/TwoInts "{a: 1, b: 2}"`
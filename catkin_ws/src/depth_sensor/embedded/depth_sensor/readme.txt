Mar 31 2023

We have had issues with getting rosserial arduino to cooperate so I'm writing this readme
to explain how to get it to work. Firstly, it is only possible to build and flash with catkin for certain boards.
I know the uno and the mega work but the due doesn't, and this is because the due has a different architecture. Same
goes for any adafruit products. If you have a compatible baord, refer to the imu package for how to create a 
package for this.

If you can't use catkin to build or flash for any reason, you need to build and flash through the arduino ide.
First step is to go to the rosserial arduino tutorial site. On the site it tells you to delete and rebuild the 
libraries. You need to do this otherwise it won't work. Then, get the hello world exmaple to work. Finally, transplant
your code into the hello world program one step at a time until the whole thing works.

The default baud is 57600. To change this call nh.getHardware->setbaud();
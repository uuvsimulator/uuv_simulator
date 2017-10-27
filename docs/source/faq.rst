.. _faq:

F.A.Q.
======

- **How do I find out the ID number my joystick?**

You can use **jstest-gtk** to find that out. To install it, run ::

  sudo apt-get install jstest-gtk

and run it on the terminal. It will open a window as seen below. The joystick
devices are named **/dev/input/js1** for the device #1. You can use this index
for the launch files that have the input option **joy_id**.

.. image:: images/jstest.png

- **My machine doesn't seem to see the services and topics from the roscore running remotely.**

If you are having trouble running your application in multiple computers, you
can try adding the hostnames and IP addresses of your machines to the
**/etc/hosts** file of the computer running **roscore**.

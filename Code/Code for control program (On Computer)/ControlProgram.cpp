//sudo chmod 0666 /dev/ttyUSB1
//$ Command to find serial port reqd :  ls /sys/bus/usb-serial/devices/ -ltrah
//    #include <stdio.h>   /* Standard input/output definitions */
	#include <iostream>
    #include <string.h>  /* String function definitions */
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <stdlib.h>
using namespace std;
    #include <fstream>


/* This function opens the serial port 'ttyUSB0'*/
int open_port(void)
    {
    int fd;
      fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
      if (fd == -1)
        {
            perror("open_port: Unable to open /dev/ttyUSB0 - ");
        }
      else
            fcntl(fd, F_SETFL, 0);

      return (fd);
    }

/* Reads one character from the serial port */
char read_port(void)
    {
        int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);
        if (fd == -1)
            {
                /* Could not open the port. */
                perror("open_port: Unable to open /dev/ttyUSB0 - ");
            }
	
        char buffer;
        int n = read(fd, &buffer, 1);
        if (n < 0)
            {fputs("read failed!\n", stderr);printf("not read\n");}

        //printf("%c\n", buffer);

        return (buffer);
    }


/* Writes one character to serial port */
int write_port(int fd,unsigned char comm)
	{
		char buf[10000];
		unsigned char hchk[1],vchk[1],ofst[1];
		FILE *file;
		int n,a;		
		char* pch;	
		 int i=0;
		hchk[0] = (unsigned char)comm;
		n = write(fd,hchk, 1);
		if (n < 0)
			fputs("write() of 1 bytes failed!\n", stderr); 					
   		return 0;
	}


int main()
{
    int fd;
	ifstream field; /* For reading file for field plan */
	field.open("field.txt");
    struct termios tty;
    struct termios tty_old;
    memset (&tty, 0, sizeof tty);
    fd=open_port();
    if ( tcgetattr ( fd, &tty ) != 0 )
        {
            printf("Error ");
        }
    /* Save old tty parameters */
    tty_old = tty;
    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B9600);
    cfsetispeed (&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;        // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_cflag     |=  CS8;

    tty.c_cflag     &=  ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN]      =   1;                  // read doesn't block
    tty.c_cc[VTIME]     =   5;                  // 0.5 seconds read timeout
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );
    if ( tcsetattr ( fd, TCSANOW, &tty ) != 0)
        {
            printf ("Error " );
        }
//////////////////////////////////////////////////////////////////////////////////////////////4
	int dist;
	unsigned char dia;
	int breakpoint = 0;
	bool newtrough = 0;
	unsigned char inc;
	int done=0;
	int endline1,endline2;
	int dist_temp;
	field >> endline1 >>endline2; /* Line1 and Line2 end at checkpoints 'endline1' and endline2' respectively */
	
	while( field >> dist ) /* Read the distance for Holes (Checkpoints are at distance 20 cm from each other)  */
	{
		write_port(fd,'g'); /* Send command 'g' to start moving */
		dist_temp = dist/20;
		while( breakpoint < dist_temp )  /* Loop until the desired checkpoint is reached */
		{
			char c = read_port(); /* Read the response from bot */
			cout << c << "is rcvd" <<endl;
			if( c == 'p' )		/* If response is for checkpoint encountered */
			{
				cout << "p inside\n";
				breakpoint++;	/* Increase the breakpoint count */
			}
			
			cout <<" brkpnt : "<<breakpoint<<"\n";
			if( breakpoint == endline1 || breakpoint == endline2 )	/* If end of line is reached */
			{	
				cout<<"Endline1\n";			
				write_port(fd,'6');		/* Send command to turn right */
				cout<<"Endline1\n";
				cout << read_port() << endl;	/* Receive confirmation that turn is complete */
				
				write_port(fd,'g');		/* send command to move */
				
				if(read_port()=='p')		/* If breakpoint encountered , skip that breakpoint*/
					{
					   cout<<" 1st checkpoint skipped"<<endl;	
						if (read_port()=='p')	/* one more checkpoint */
							{
								 cout<<" 2nd checkpoint skipped"<<endl;
								 write_port(fd,'6');	/* Turn Right again */
								 cout<<"2nd turn"<<read_port()<<endl;	/* receive confirmation that turn is complete */
							}	 
					}
				if ( breakpoint == endline2 ) /* If bot completes two lines */
					{
						write_port(fd,'0'); /*Stop*/
						return 0;
					}
			}
			
			cout << "My Check 5 \n";	
		}
		
		field >> dia;	/* Input reqd diameter of hole */
		write_port(fd,dia);	/* send diameter to bot */
		write_port(fd,'3');	/*send command 3 to start digging */
		cout << read_port()<<endl;	/* receive acknowlegemet that digging is complete */
		cout << "Hole dug"<<endl;
		write_port(fd, '1');		/* send command to rotate the bot */
		cout<<"should give 1. gives "<<read_port()<<endl; /* receive acknowlegement that bot has turne */
		cout << "Turned" <<endl; 
		sleep(5);
		system("sudo ./takepic.sh");	/* Take picture and start image processing */
	/*************************************************************/
		int rad;					 // Code to verify the radius and start digging again if not satisfied.
		bool digAgain = false;
		ifstream camera;
		camera.open("radius.txt");
		camera >> rad;
		camera.close();
		switch(dia)
		{
			case 'a' : if(rad < 40)
					digAgain = true;
					break;
			case 'b' : if(rad < 90)
					digAgain = true;
					break;
			case 'c' : if (rad < 140)		//These compared values need to be calibrated according to the camera
					digAgain = true;		// resolution and distance of camera from hole.
					break;
			case 'd' : if (rad < 190)		
					digAgain = true;
					break;
			case 'e' : if (rad < 240)
					digAgain = true;
					break;
		}

		if (digAgain) // If diameter is shorter than required, dig again.
		{
			write_port(fd,'3');
			cout << read_port()<<endl;	
			cout << "Hole dug"<<endl;
		}
	/*************************************************************/
		sleep(5);
	}	
	

	

	//read_port();
    //write_port(fd,'3');

    //fcntl(fd, F_SETFL, O_NDELAY);
    


close(fd);
}


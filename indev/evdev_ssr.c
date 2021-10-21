/**
 * @file evdev.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "evdev.h"
#if defined(USE_SSR_EVDEV) && USE_SSR_EVDEV

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>
#include <errno.h>


/*********************
 *      DEFINES
 *********************/
#define MOUSE_DEV_NAME   "/dev/input/mice"

/* Mouse button bits*/
#define WHEEL_UP    0x10
#define WHEEL_DOWN  0x08

#define BUTTON_L    0x01
#define BUTTON_M    0x02
#define BUTTON_R    0x03
#define SCALE       1 /* default scaling factor for acceleration */
#define THRESH      1 /* default threshhold for acceleration */


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void mouse_setrange (int newminx, int newminy, int newmaxx, int newmaxy);
static int mouse_update(int dx, int dy, int dz);
static int IMPS2_Read(int fd, int *dx, int *dy, int *dz, int *bp);
static void *ST_FBMouseUserProc(void *args);

/**********************
 *  STATIC VARIABLES
 **********************/
pthread_t g_mouse_user_pt;

static int xpos; /* current x position of mouse */
static int ypos; /* current y position of mouse */
static int minx; /* minimum allowed x position */
static int maxx; /* maximum allowed x position */
static int miny; /* minimum allowed y position */
static int maxy; /* maximum allowed y position */
static int button; /* current state of buttons */

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize the evdev interface
 */
void evdev_init(int screen_width, int screen_height)
{
    mouse_setrange(0, 0, screen_width, screen_height);

    ST_Fb_InitMouse(32, 32, 2, (unsigned char*)"./cursor-arrow.dat");
    ST_Fb_MouseSet(screen_width/2, screen_height/2);

    pthread_create(&g_mouse_user_pt, NULL, ST_FBMouseUserProc, NULL);
}
/**
 * reconfigure the device file for evdev
 * @param dev_name set the evdev device filename
 * @return true: the device file set complete
 *         false: the device file doesn't exist current system
 */
bool evdev_set_file(char* dev_name)
{ 
     return false;
}
/**
 * Get the current position and state of the evdev
 * @param data store the evdev data here
 * @return false: because the points are not buffered, so no more data to be read
 */
void evdev_read(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    /*Store the collected data*/
    data->point.x = xpos;
    data->point.y = ypos;
    data->state = (button == BUTTON_L) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    return ;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
static void mouse_setrange (int newminx, int newminy, int newmaxx, int newmaxy)
{
    minx = newminx;
    miny = newminy;
    maxx = newmaxx;
    maxy = newmaxy;
}

static int mouse_update(int dx, int dy, int dz)
{
    int sign;

    sign = 1;
    if (dx < 0)
    {
        sign = -1;
        dx = -dx;
    }

    if (dx > THRESH)
        dx = THRESH + (dx - THRESH) * SCALE;

    dx *= sign;
    xpos += dx;

    if( xpos < minx )
        xpos = minx;
    if( xpos > maxx )
        xpos = maxx;

    sign = 1;
    if (dy < 0)
    {
        sign = -1;
        dy = -dy;
    }

    if (dy > THRESH)
         dy = THRESH + (dy - THRESH) * SCALE;

    dy *= sign;
    ypos += dy;

    if (ypos < miny)
        ypos = miny;

    if (ypos > maxy)
        ypos = maxy;

    return 1;
}

static int IMPS2_Read(int fd, int *dx, int *dy, int *dz, int *bp)
{
    static unsigned char buf[5];
    static int buttons[7] = {0, 1, 3, 0, 2, 0, 0};// 1:left button, 2: mid button, 3: right button
    static int nbytes = 0;
    int n;

    while ((n = read (fd, &buf [nbytes], 4 - nbytes))) {
        if (n < 0) {
            if (errno == EINTR)
                continue;
            else
                return -1;
        }

        nbytes += n;

        if (nbytes == 4) {
            int wheel;
            if ((buf[0] & 0xc0) != 0) {
                buf[0] = buf[1];
                buf[1] = buf[2];
                buf[2] = buf[3];
                nbytes = 3;
                return -1;
            }

            /* FORM XFree86 4.0.1 */
            *bp = buttons[(buf[0] & 0x07)];
            *dx = (buf[0] & 0x10) ? buf[1] - 256 : buf[1];
            *dy = (buf[0] & 0x20) ? -(buf[2] - 256) : -buf[2];

            /* Is a wheel event? */
            if ((wheel = buf[3]) != 0) {
                if(wheel > 0x7f) {
                    *bp |= WHEEL_UP;
                }
                else {
                    *bp |= WHEEL_DOWN;
                }
            }

            *dz = 0;
            nbytes = 0;
            return 1;
        }
    }
    return 0;
}

static void *ST_FBMouseUserProc(void *args)
{
    int fd,retval;
    fd_set readfds;
    unsigned char imps2_param [] = {243, 200, 243, 100, 243, 80};
    int dx, dy, dz;

    do {
        fd = open(MOUSE_DEV_NAME, O_RDWR);
        if (fd < 0) {
            printf("can not open %s\n", MOUSE_DEV_NAME);
        }
        sleep(5);
    } while (fd < 0);

    printf("open %s success, fd:%d\n", MOUSE_DEV_NAME, fd);

    write(fd, imps2_param, sizeof (imps2_param));

    while(1) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        retval = select(fd + 1, &readfds, NULL, NULL, NULL);
        if(retval == 0) {
            continue;
        }

        if(FD_ISSET(fd, &readfds)) {
            IMPS2_Read(fd, &dx, &dy, &dz, &button);
            mouse_update(dx, dy, dz);
            ST_Fb_MouseSet(xpos, ypos);
            //printf("[%04x, %04x, %04x\n]\n", xpos, ypos, button);
        }
    }
    close(fd);
    return NULL;
}

#endif

/* 
 * Adafruit 32x16 Device driver for scrolling text
 *
 * This is Experimental code
 * 2015 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kdev_t.h> /* MKDEV */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/types.h> /* dev_t type */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h> /* kmalloc */
#include <linux/kthread.h> 
#include <linux/delay.h> /* *delay */
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("muzzynine");

#define DEVICE_NAME "adafruit_r16"
#define UPDATE_THREAD "matrix_update_thread"

#define MATRIX_DEVIDE_ROW 8
#define MATRIX_COLUMN 32
#define MATRIX_PWM 8

#define BCM2835_PERIPHERAL_BASE 0x20000000
#define GPIO_REGISTER_BASE (BCM2835_PERIPHERAL_BASE + 0x200000)

#define MATRIX_GPIO_OE 2
#define MATRIX_GPIO_CLK 3
#define MATRIX_GPIO_LAT 4
#define MATRIX_GPIO_A 7
#define MATRIX_GPIO_B 8
#define MATRIX_GPIO_C 9
#define MATRIX_GPIO_D 10
#define MATRIX_GPIO_R1 17
#define MATRIX_GPIO_B1 18
#define MATRIX_GPIO_G1 22
#define MATRIX_GPIO_R2 23
#define MATRIX_GPIO_B2 24
#define MATRIX_GPIO_G2 25

#define GPIO_FSEL_INPUT 0
#define GPIO_FSEL_OUTPUT 1

#define ADAFRUIT_R16_MINOR 0
static int adafruit_r16_major = 0;

#define IO_ADDRESS(x) (((x) & 0x0fffffff) + (((x) >> 4) & 0x0f000000) + 0xf0000000)
#define __io_address(a) __io(IO_ADDRESS(a))

/* one gpio fsel register is select 10 gpio pin(0-9/10-19/20-29/30-39) */
#define GPIOFSEL(x) (0x00+(x)*4)
/* one gpio sel register is set 10 gpio pin */
#define GPIOSET(x) (0x1c+(x)*4)
#define GPIOCLR(x) (0x28+(x)*4)

#define FONT_READ_BYTE(x) (*(const unsigned char *)(x))

#define GPIO_FSEL_INPUT 0
#define GPIO_FSEL_OUTPUT 1

static unsigned char Font5x7[] = {
  0x00, 0x00, 0x00, 0x00, 0x00,/* (space) */
  0x00, 0x00, 0x5F, 0x00, 0x00,/* ! */
  0x00, 0x07, 0x00, 0x07, 0x00,/* " */
  0x14, 0x7F, 0x14, 0x7F, 0x14,/* # */
  0x24, 0x2A, 0x7F, 0x2A, 0x12,/*  $*/
  0x23, 0x13, 0x08, 0x64, 0x62,/* % */
  0x36, 0x49, 0x55, 0x22, 0x50,/* & */
  0x00, 0x05, 0x03, 0x00, 0x00,/* ' */
  0x00, 0x1C, 0x22, 0x41, 0x00,/* ( */
  0x00, 0x41, 0x22, 0x1C, 0x00,/* ) */
  0x08, 0x2A, 0x1C, 0x2A, 0x08,/* * */
  0x08, 0x08, 0x3E, 0x08, 0x08,/* + */
  0x00, 0x50, 0x30, 0x00, 0x00,/* , */
  0x08, 0x08, 0x08, 0x08, 0x08,/* - */
  0x00, 0x60, 0x60, 0x00, 0x00,/* . */
  0x20, 0x10, 0x08, 0x04, 0x02,/* / */
  0x3E, 0x51, 0x49, 0x45, 0x3E,/* 0 */
  0x00, 0x42, 0x7F, 0x40, 0x00,/* 1 */
	0x42, 0x61, 0x51, 0x49, 0x46,// 2
	0x21, 0x41, 0x45, 0x4B, 0x31,// 3
	0x18, 0x14, 0x12, 0x7F, 0x10,// 4
	0x27, 0x45, 0x45, 0x45, 0x39,// 5
	0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
	0x01, 0x71, 0x09, 0x05, 0x03,// 7
	0x36, 0x49, 0x49, 0x49, 0x36,// 8
	0x06, 0x49, 0x49, 0x29, 0x1E,// 9
	0x00, 0x36, 0x36, 0x00, 0x00,// :
	0x00, 0x56, 0x36, 0x00, 0x00,// ;
	0x00, 0x08, 0x14, 0x22, 0x41,// <
	0x14, 0x14, 0x14, 0x14, 0x14,// =
	0x41, 0x22, 0x14, 0x08, 0x00,// >
	0x02, 0x01, 0x51, 0x09, 0x06,// ?
	0x32, 0x49, 0x79, 0x41, 0x3E,// @
	0x7E, 0x11, 0x11, 0x11, 0x7E,// A
	0x7F, 0x49, 0x49, 0x49, 0x36,// B
	0x3E, 0x41, 0x41, 0x41, 0x22,// C
	0x7F, 0x41, 0x41, 0x22, 0x1C,// D
	0x7F, 0x49, 0x49, 0x49, 0x41,// E
	0x7F, 0x09, 0x09, 0x01, 0x01,// F
	0x3E, 0x41, 0x41, 0x51, 0x32,// G
	0x7F, 0x08, 0x08, 0x08, 0x7F,// H
	0x00, 0x41, 0x7F, 0x41, 0x00,// I
	0x20, 0x40, 0x41, 0x3F, 0x01,// J
	0x7F, 0x08, 0x14, 0x22, 0x41,// K
	0x7F, 0x40, 0x40, 0x40, 0x40,// L
	0x7F, 0x02, 0x04, 0x02, 0x7F,// M
	0x7F, 0x04, 0x08, 0x10, 0x7F,// N
	0x3E, 0x41, 0x41, 0x41, 0x3E,// O
	0x7F, 0x09, 0x09, 0x09, 0x06,// P
	0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
	0x7F, 0x09, 0x19, 0x29, 0x46,// R
	0x46, 0x49, 0x49, 0x49, 0x31,// S
	0x01, 0x01, 0x7F, 0x01, 0x01,// T
	0x3F, 0x40, 0x40, 0x40, 0x3F,// U
	0x1F, 0x20, 0x40, 0x20, 0x1F,// V
	0x7F, 0x20, 0x18, 0x20, 0x7F,// W
	0x63, 0x14, 0x08, 0x14, 0x63,// X
	0x03, 0x04, 0x78, 0x04, 0x03,// Y
	0x61, 0x51, 0x49, 0x45, 0x43,// Z
	0x00, 0x00, 0x7F, 0x41, 0x41,// [
	0x02, 0x04, 0x08, 0x10, 0x20,// "\"
	0x41, 0x41, 0x7F, 0x00, 0x00,// ]
	0x04, 0x02, 0x01, 0x02, 0x04,// ^
	0x40, 0x40, 0x40, 0x40, 0x40,// _
	0x00, 0x01, 0x02, 0x04, 0x00,// `
	0x20, 0x54, 0x54, 0x54, 0x78,// a
	0x7F, 0x48, 0x44, 0x44, 0x38,// b
	0x38, 0x44, 0x44, 0x44, 0x20,// c
	0x38, 0x44, 0x44, 0x48, 0x7F,// d
	0x38, 0x54, 0x54, 0x54, 0x18,// e
	0x08, 0x7E, 0x09, 0x01, 0x02,// f
	0x08, 0x14, 0x54, 0x54, 0x3C,// g
	0x7F, 0x08, 0x04, 0x04, 0x78,// h
	0x00, 0x44, 0x7D, 0x40, 0x00,// i
	0x20, 0x40, 0x44, 0x3D, 0x00,// j
	0x00, 0x7F, 0x10, 0x28, 0x44,// k
	0x00, 0x41, 0x7F, 0x40, 0x00,// l
	0x7C, 0x04, 0x18, 0x04, 0x78,// m
	0x7C, 0x08, 0x04, 0x04, 0x78,// n
	0x38, 0x44, 0x44, 0x44, 0x38,// o
	0x7C, 0x14, 0x14, 0x14, 0x08,// p
	0x08, 0x14, 0x14, 0x18, 0x7C,// q
	0x7C, 0x08, 0x04, 0x04, 0x08,// r
	0x48, 0x54, 0x54, 0x54, 0x20,// s
	0x04, 0x3F, 0x44, 0x40, 0x20,// t
	0x3C, 0x40, 0x40, 0x20, 0x7C,// u
	0x1C, 0x20, 0x40, 0x20, 0x1C,// v
	0x3C, 0x40, 0x30, 0x40, 0x3C,// w
	0x44, 0x28, 0x10, 0x28, 0x44,// x
	0x0C, 0x50, 0x50, 0x50, 0x3C,// y
	0x44, 0x64, 0x54, 0x4C, 0x44,// z
	0x00, 0x08, 0x36, 0x41, 0x00,// {
	0x00, 0x00, 0x7F, 0x00, 0x00,// |
	0x00, 0x41, 0x36, 0x08, 0x00,// }
	0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
	0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};

struct color {
  u8 red;
  u8 green;
  u8 blue;
};

struct adafruit_r16_gpio {
  void __iomem *base;
};


struct adafruit_r16_device {
  struct device *class_dev;
  struct cdev cdev;
  struct class *class;
  struct adafruit_r16_gpio gpio;
};


struct matrix_element {
    unsigned int r1:1;
    unsigned int g1:1;
    unsigned int b1:1;
    unsigned int r2:1;
    unsigned int g2:1;
    unsigned int b2:1;
};

struct matrix_setting {
  int text_cursor_x;
  int text_cursor_y;
  int font_width;
  int font_height;
  struct color color;
};

struct adafruit_r16_matrix_buffer {
  struct matrix_setting setting;
  struct matrix_element panel[MATRIX_PWM][MATRIX_DEVIDE_ROW][MATRIX_COLUMN];
};
  

static struct adafruit_r16_device *led_matrix_device = NULL;
static struct adafruit_r16_matrix_buffer *led_matrix_buffer = NULL;
static struct task_struct *update_thread_id = NULL;
static struct task_struct *ctl_thread_id = NULL;
static char output_str[200];
static int output_str_length;
static spinlock_t s_lock;


ssize_t adafruit_r16_write(struct file *f, const char __user *buf, size_t count, loff_t *f_pos){
  int retval = 1;
  unsigned char data[200];

  printk(KERN_ALERT "WRITE");

  if(copy_from_user(data, buf, count)){
    retval = -EFAULT;
    return count;
  }

  spin_lock(&s_lock);
  
  memset(output_str, 0, 200);
  memcpy(output_str, data, count);
  output_str_length = count;
  
  spin_unlock(&s_lock);
  
  return count;
}

int adafruit_r16_open(struct inode *inode, struct file *f){
  return 0;
}
  
int adafruit_r16_close(struct inode *inode, struct file *f){
  return 0;
}

struct file_operations adafruit_r16_fops = {
 .write = adafruit_r16_write,
 .open = adafruit_r16_open,
 .release = adafruit_r16_close,
};

static void gpio_write(struct adafruit_r16_gpio *gpio, unsigned offset, int value){
  unsigned gpio_fset_register = offset / 32;
  unsigned gpio_fset_field_offset = (offset - 32 * gpio_fset_register);

  //  printk(KERN_DEBUG "\ngpio_write offset : %d register : %d field_offset : %d\n", offset, gpio_fset_register, gpio_fset_field_offset);


  if(value){
    writel(1 << gpio_fset_field_offset, gpio->base + GPIOSET(gpio_fset_register));
  } else {
    writel(1 << gpio_fset_field_offset, gpio->base + GPIOCLR(gpio_fset_register));
  }
}

static void gpio_dir_out(struct adafruit_r16_gpio *gpio, unsigned offset, int value){
  unsigned gpiodir;
  unsigned gpio_fsel_register = offset / 10;
  unsigned gpio_fsel_field_offset = (offset - 10 * gpio_fsel_register) * 3;

  printk(KERN_DEBUG "\ngpio_dir_out offset : %d register : %d field_offset : %d\n", offset, gpio_fsel_register, gpio_fsel_field_offset);

  gpiodir = readl(gpio->base + GPIOFSEL(gpio_fsel_register));

  //해당 offset에 대해서는 clear
  gpiodir &= ~(7 << gpio_fsel_field_offset);

  //pin output(001)
  gpiodir |= 1 << gpio_fsel_field_offset;
  writel(gpiodir, gpio->base + GPIOFSEL(gpio_fsel_register));

  gpiodir = readl(gpio->base + GPIOFSEL(gpio_fsel_register));
  /*
  int i =31;

  for(i ; i>= 0 ; --i){
    printk(KERN_DEBUG "%d", ((int)gpiodir >> i) & 1);
  }
  */

  gpio_write(gpio, offset, value);
}


static void gpio_init(struct adafruit_r16_gpio *gpio){


  gpio_dir_out(gpio, MATRIX_GPIO_OE, 1);
  gpio_dir_out(gpio, MATRIX_GPIO_LAT, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_CLK, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_A, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_B, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_C, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_D, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_R1, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_R2, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_B1, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_B2, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_G1, 0);
  gpio_dir_out(gpio, MATRIX_GPIO_G2, 0);

  mdelay(15);
}

static void update(void){
  int row;
  for(row = 0 ; row < MATRIX_DEVIDE_ROW ; row++){
    int pwm_i;
    for(pwm_i = 0; pwm_i < MATRIX_PWM; pwm_i++){
      int col;
      for(col = 0; col < MATRIX_COLUMN; col++){
	struct matrix_element selected = led_matrix_buffer->panel[pwm_i][row][col];

	//all gpio pin clear
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_CLK, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_R1, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B1, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_G1, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_R2, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_G2, 0);
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B2, 0);

	ndelay(250);

	if(selected.r1){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_R1, 1);
	}
	if(selected.r2){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_R2, 1);
	}
	if(selected.g1){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_G1, 1);
	}
	if(selected.g2){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_G2, 1);
	}
	if(selected.b1){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B1, 1);
	}
	if(selected.b2){
	  gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B2, 1);
	}

	//	printk(KERN_DEBUG "%d %d %d %d %d %d", selected.r1, selected.r2, selected.b1, selected.b2, selected.g1, selected.g2);

	ndelay(250);

	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_CLK, 1);

	ndelay(250);
      }

      gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_OE, 1);

      if((row & (1 << 2)) == (1 << 2)){
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_C, 1);
      } else {
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_C, 0);
      }
      if((row & (1 << 1)) == (1 << 1)){
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B, 1);
      } else {
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_B, 0);
      }	
      if((row & 1) == 1){
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_A, 1);
      } else {
	gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_A, 0);
      }

      gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_LAT, 1);
      gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_LAT, 0);

      gpio_write(&led_matrix_device->gpio, MATRIX_GPIO_OE, 0);

      ndelay(3400);
    }
  }
}

static void draw(u8 x, u8 y, struct color color){
  u8 red = color.red;
  u8 green = color.green;
  u8 blue = color.blue;

  int i;
  for(i = 0 ; i < MATRIX_PWM ; i++){
    u8 mask = 1<<i;
    struct matrix_element *selected = &led_matrix_buffer->panel[i][y][x];

    if(y < 8){
      selected->r1 = (red & mask) == mask;
      selected->g1 = (green & mask) == mask;
      selected->b1 = (blue & mask) == mask;
    } else {
      selected->r2 = (red & mask) == mask;
      selected->g2 = (green & mask) == mask;
      selected->b2 = (blue & mask) == mask;
    }
  }
}


static void writeChar(char c, struct matrix_setting *setting){
  //  printk(KERN_ALERT "%c ", c);
  
  if(c == '\n'){
    setting->text_cursor_x = 0;
    setting->text_cursor_y += setting->font_height;
  } else {
    int i = 0;
    for(i = 0; i < setting->font_width+1 ; i++){
      u8 line;

      if(i == setting->font_width || (setting->text_cursor_x < 0 || setting->text_cursor_x > 32)){ //공백
	line = 0x0;
      } else {

	line = FONT_READ_BYTE(Font5x7 + ((c - 0x20) * setting->font_width) + i); //한줄

      }

      int j = 0;

      for(j = 0; j < setting->font_height+1 ; j++){
	if(line & 0x1){
	  draw(setting->text_cursor_x, setting->text_cursor_y+j, setting->color);
	}
	line >>= 1;
      }

      setting->text_cursor_x += 1;
    }
  }
}


static void writeText(struct adafruit_r16_matrix_buffer *buffer){
  int i;

  for(i = 0; i < output_str_length-1; i++){
    writeChar(output_str[i], &buffer->setting);
  }
}

static void loop(void){
  char str[] = "TESTING";
  int cursor_x = MATRIX_COLUMN;
  int cursor_min = sizeof(str) * -12;

  led_matrix_buffer->setting.text_cursor_x = cursor_x;
  //  writeText(str);

  if((--cursor_x) < cursor_min) cursor_x = MATRIX_COLUMN;

}

static void fill_blank(void){
  int i;
  int j;
  int k;
  for(i = 0; i < MATRIX_PWM ; i++){
    for(j = 0; j < MATRIX_DEVIDE_ROW ; j++){
      for(k = 0 ; k < MATRIX_COLUMN ; k++){
	struct matrix_element *selected = &led_matrix_buffer->panel[i][j][k];

	selected->r1 = 0;
	selected->r2 = 0;
	selected->b1 = 0;
	selected->b2 = 0;
	selected->g1 = 0;
	selected->g2 = 0;
	
      }
    }
  }
}


static int thread_update_job(void *args){
  printk(KERN_DEBUG "thread update job");
  int text_x = MATRIX_COLUMN;
  int text_min;
  
  while(!kthread_should_stop()){
    schedule();

    fill_blank();

    led_matrix_buffer->setting.text_cursor_x = text_x;

        spin_lock(&s_lock);

        text_min = output_str_length * -12;
    writeText(led_matrix_buffer);

    spin_unlock(&s_lock);

    update();

    if((--text_x) < text_min) text_x = MATRIX_COLUMN;
    mdelay(20);
  }
  printk(KERN_DEBUG "kthread_should_stop() called");
  return 0;
}

static int thread_control_job(void *args){
  while(!kthread_should_stop()){
    loop();
  }
  return 0;
}

static int kthread_init(void){
  printk(KERN_ALERT "init kernal thread\n");

  /*
  if(ctl_thread_id == NULL){
    ctl_thread_id = (struct task_struct *)kthread_run(thread_control_job, NULL, "display control thread");
  }
  */

  if(update_thread_id == NULL){
    update_thread_id = (struct task_struct *)kthread_run(thread_update_job, NULL, UPDATE_THREAD);
    if(IS_ERR(update_thread_id)){
      printk(KERN_INFO "unable to start kernel thread\n");
      update_thread_id= NULL;

      return -ENOMEM;
    }
  }
  return 0;
}


static void print_matrix(void){
  int i;
  int j;
  int k;
  for(i = 0; i < MATRIX_PWM ; i++){
    for(j = 0; j < MATRIX_DEVIDE_ROW ; j++){
      for(k = 0 ; k < 5 ; k++){
	struct matrix_element *selected = &led_matrix_buffer->panel[i][j][k];

		printk(KERN_DEBUG "%d %d %d %d %d %d %d %d %d", i, j, k, selected->r1, selected->g1, selected->b1, selected->r2, selected->g2, selected->b2);
	
      }
    }
  }
}

static ssize_t led_matrix_show(struct device *dev, struct device_attribute *attr, char *buf){
  return 0;
}

static ssize_t led_matrix_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
  return 0;
}
/*
static struct device_attribute led_matrix_class_attrs[] = {
  __ATTR(control, 0666, led_matrix_show, led_matrix_store),
  __ATTR_NULL,
};
*/
static int adafruit_r16_init(void){
  dev_t dev;
  int major;
  int alloc_err = 0;
  int cdev_err = 0;
  struct color white;
  white.red = 255;
  white.green = 255;
  white.blue = 255;

  printk(KERN_ALERT "init\n");
  dev = MKDEV(adafruit_r16_major, ADAFRUIT_R16_MINOR);

  //반드시 할
   led_matrix_device = kmalloc(sizeof(struct adafruit_r16_device), GFP_KERNEL);
   if(led_matrix_device == NULL){
     return -ENOMEM;
   }
   led_matrix_buffer = kmalloc(sizeof(struct adafruit_r16_matrix_buffer), GFP_KERNEL);
   if(led_matrix_buffer == NULL){
     return -ENOMEM;
   }


  //Set gpio base address
  led_matrix_device->gpio.base = __io_address(GPIO_REGISTER_BASE);
  printk(KERN_DEBUG "gpio base : %X", led_matrix_device->gpio.base);


  //register char device(allocate dynamical major number, minor number)
  alloc_err = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);

  if(alloc_err < 0){
    kfree(led_matrix_device);
    return -1;
  }
  
  adafruit_r16_major = major = MAJOR(dev);

  //initialize a cdev structure
  cdev_init(&(led_matrix_device->cdev), &adafruit_r16_fops);
  led_matrix_device->cdev.owner = THIS_MODULE;
  led_matrix_device->cdev.ops = &adafruit_r16_fops;


  /*add a char device to system*/
  cdev_err = cdev_add(&(led_matrix_device->cdev), MKDEV(adafruit_r16_major, ADAFRUIT_R16_MINOR), 1);

  if(cdev_err < 0){
    unregister_chrdev_region(dev, 1);
    kfree(led_matrix_device);
    kfree(led_matrix_buffer);
    return -1;
  }

 
    led_matrix_device->class = class_create(THIS_MODULE, DEVICE_NAME);
  if(IS_ERR(led_matrix_device->class)){
    cdev_del(&(led_matrix_device->cdev));
    unregister_chrdev_region(dev, 1);
    kfree(led_matrix_device);
    kfree(led_matrix_buffer);
    return -1;
  }

  //  led_matrix_device->class->dev_attrs = led_matrix_class_attrs;
  led_matrix_device->class_dev = device_create(led_matrix_device->class, NULL, MKDEV(adafruit_r16_major, ADAFRUIT_R16_MINOR), NULL, "adafruit_r16%d", ADAFRUIT_R16_MINOR);

  gpio_init(&led_matrix_device->gpio);

  //text buffer default setting
  led_matrix_buffer->setting.text_cursor_x = 0;
  led_matrix_buffer->setting.text_cursor_y = 5;
  led_matrix_buffer->setting.font_width = 5;
  led_matrix_buffer->setting.font_height = 7;
  led_matrix_buffer->setting.color = white;


  spin_lock(&s_lock);
  char waiting[] = "WAITING...";
  memset(output_str, 0, 200);
  memcpy(output_str, waiting, sizeof(waiting));
  output_str_length = (int)sizeof(waiting);

  spin_unlock(&s_lock);

  printk(KERN_ALERT "init string : %s\n", output_str);

  fill_blank();

  ndelay(250);

  kthread_init();

  // print_matrix();
  printk(KERN_ALERT "adafruit_r16 driver(major %d) installed\n", major);
  
  return 0;
}


static void adafruit_r16_exit(void){
  dev_t dev = MKDEV(adafruit_r16_major, ADAFRUIT_R16_MINOR);
  printk(KERN_ALERT "driver realease\n");

    /*
  if(ctl_thread_id){
    kthread_stop(ctl_thread_id);
    ctl_thread_id = NULL;
    }*/

  if(update_thread_id){
    printk(KERN_INFO "kill kernel thread\n");
    kthread_stop(update_thread_id);
    update_thread_id = NULL;
  } else {
    printk(KERN_INFO "no kernel thread to kill\n");
  }
  

  device_destroy(led_matrix_device->class, dev);
  class_destroy(led_matrix_device->class);
  cdev_del(&(led_matrix_device->cdev));
  unregister_chrdev_region(dev, 1);

  kfree(led_matrix_device);
  kfree(led_matrix_buffer);

  printk(KERN_DEBUG "adafruit_r16 removed\n");
		
}

module_init(adafruit_r16_init);
module_exit(adafruit_r16_exit);
  

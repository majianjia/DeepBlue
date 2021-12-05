/*
 * File      : fs_test.c
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-11-6     majianjia      first version
 */

#include <rtthread.h>
#include <dfs_posix.h>

#define TEST_FILE_SIZE_MAX 2048000

static int writespeed(int fd, char *buff_ptr, unsigned int total_length, int block_size)
{
    int index, length;
    rt_tick_t tick;
	
	/* prepare write data */
	for (index = 0; index < block_size; index++)
	{
		buff_ptr[index] = index;
	}
	index = 0;

	/* get the beginning tick */
    tick = rt_tick_get();
	while (index < total_length / block_size)
	{
		length = write(fd, buff_ptr, block_size);
		if (length != block_size)
		{
			rt_kprintf("write failed\n");
			break;
		}

		index ++;
	}
    tick = rt_tick_get() - tick;

    /* calculate write speed */
    return (total_length / tick * RT_TICK_PER_SECOND);

}

static int readspeed(int fd, char *buff_ptr, rt_size_t total_length ,int block_size)
{
    rt_tick_t tick;
	
    tick = rt_tick_get();
    total_length = 0;
    while (1)
    {
        int length;
        length = read(fd, buff_ptr, block_size);

        if (length <= 0) break;
        total_length += length;
    }
    tick = rt_tick_get() - tick;
	
    /* calculate read speed */
	return (total_length / tick * RT_TICK_PER_SECOND);
}

void fs_test(const char* filename)
{
	unsigned int i = 512;
	unsigned int speed;
	float speed_mb[32][2];  //read and write speed
	
	int fd;
	char *buff_ptr;
	rt_size_t total_length;			//file length
	unsigned int readspeed_avg = 0;
	unsigned int writespeed_avg = 0;
	unsigned int speed_count = 0;
	
	rt_kprintf("\nStart test...\n");
	
	for(i = 512;i < 4096000 ; i+= i)
	{
		total_length = 256*i;
		if(total_length > TEST_FILE_SIZE_MAX)
			total_length = TEST_FILE_SIZE_MAX;
			
		//write speed test
		fd = open(filename,  O_WRONLY | O_CREAT | O_TRUNC, 0);
		if (fd < 0)
		{
			rt_kprintf("open file:%s failed\n", filename);
			break;
		}

		buff_ptr = rt_malloc(i);	
		if (buff_ptr == RT_NULL)
		{
			rt_kprintf("no more RAM for test.\n");
			close(fd);
			break;
		}

		rt_kprintf("BlockSize:%6d Byte ->| ", i);
		
		speed = writespeed(fd, buff_ptr, total_length,i);		
		rt_kprintf("WriteSpeed:%8d byte/s ", speed);
		writespeed_avg += speed;
		speed_mb[speed_count][0] = (float)speed/(1024.f*1024.f); //writespeed in float
		
		close(fd);
		rt_free(buff_ptr);
		
		//read speed test
		fd = open(filename,  O_RDONLY, 0);
		if (fd < 0)
		{
			rt_kprintf("open file:%s failed\n", filename);
			break;
		}

		buff_ptr = rt_malloc(i);	
		if (buff_ptr == RT_NULL)
		{
		    rt_kprintf("no memory\n");
			close(fd);
			break;
		}
			
		speed = readspeed(fd, buff_ptr, total_length, i);
		rt_kprintf("| ReadSpeed:%8d byte/s\n", speed);
		readspeed_avg += speed;
		speed_mb[speed_count][1] = (float)speed/(1024.f*1024.f); //read_speed in float
		
		close(fd);
		rt_free(buff_ptr);
		
		//
		speed_count ++;
	}
	
	//draw graph
	{
		char buf[32];
		int block_max = i;
		int block = 0;
		int count_of_chars;
		float max =  speed_mb[speed_count-1][1];
		float scale = (1.f/max)*50.f;
		
		speed_count = 0;
		
		rt_kprintf("\nBlock Size (byte)        Writespeed = \"WR\"   Readspeed = \"RD\"\n");
	
		
		for(block=512; block<block_max; block+=block)
		{
			//write speed
			rt_kprintf("%6d->| ",block);
			count_of_chars = 1+ speed_mb[speed_count][0] * scale;
			while(count_of_chars--)
			{
			//	rt_thread_delay(15);
				rt_kprintf(">");
			}

			sprintf(buf, " WR:%3.2f MB/s\n", speed_mb[speed_count][0]);
			rt_kprintf(buf);
						
			//readspeed
			rt_kprintf("      ->| ",block);	
			count_of_chars = 1+ speed_mb[speed_count][1] * scale;
			while(count_of_chars--)	
			{		
				//rt_thread_delay(15);				
				rt_kprintf(">");
			}

			sprintf(buf, " RD:%3.2f MB/s\n", speed_mb[speed_count][1]);
			rt_kprintf(buf);
			if(block < block_max/2)
				rt_kprintf("        | \n");	
			
			speed_count ++;	
		}
		
	}
	
	if(speed_count > 0)
		rt_kprintf("Write Speed AVG: %d Byte/s | Read Speed AVG: %d Byte/s\n",writespeed_avg/speed_count, readspeed_avg/speed_count);
	
	unlink(filename);
}


#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(fs_test, speed vs block size);
#endif




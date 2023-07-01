#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Key.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"

void init_drv_Main()
{
	// LED鍒濆鍖栧嚱鏁�
	init_drv_LED();
	// 鍒濆鍖栧睆骞�
	init_drv_LCD();
	// 鍒濆鍖朑UI鐣岄潰
	init_GUI();
	// 鍒濆鍖栨寜閿�
	init_drv_Key();
}
#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Key.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"

void init_drv_Main()
{
	// LED初始化函敄1�7
	init_drv_LED();
	// 初始化屏幄1�7
	init_drv_LCD();
	// 初始化GUI界面
	init_GUI();
	// 初始化按锄1�7
	init_drv_Key();
}
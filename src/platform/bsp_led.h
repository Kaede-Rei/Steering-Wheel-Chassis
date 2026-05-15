#ifndef _bsp_led_h_
#define _bsp_led_h_

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief LED 单例用户自定义名称
 */
#define led led_instance

/**
 * @brief LED 单例接口
 * @param on 打开 LED 的函数指针
 * @param off 关闭 LED 的函数指针
 * @param toggle 翻转 LED 状态的函数指针
 */
extern const struct LedInterface {
    /**
     * @brief 打开 LED
     */
    void(*on)(void);

    /**
     * @brief 关闭 LED
     */
    void(*off)(void);

    /**
     * @brief 翻转 LED 状态
     */
    void(*toggle)(void);
} led_instance;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void led_on(void);
void led_off(void);
void led_toggle(void);

#endif
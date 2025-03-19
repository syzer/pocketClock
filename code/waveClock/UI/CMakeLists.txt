SET(SOURCES ui_Screen1.c
    ui_setMenu.c
    ui_setTime.c
    ui_setDate.c
    ui_Screen2.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_602517921.c
    ui_img_1415964700.c
    ui_img_683590454.c
    ui_img_878922401.c
    fonts/ui_font_date.c
    fonts/ui_font_day.c
    fonts/ui_font_digits.c
    fonts/ui_font_secs.c)

add_library(ui ${SOURCES})

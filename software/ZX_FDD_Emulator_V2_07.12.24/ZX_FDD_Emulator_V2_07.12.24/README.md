/*
# ZX_FDD_Emulator V2.0

Эмулятор дисковода от EvgenRu, ссылка на форум:
https://zx-pk.ru/threads/26328-planiruyu-sdelat-fdd-emulyator-na-atmega8.html

Ссылка на гитхаб:
https://github.com/EvgeniyRU/ZX_FDD_Emulator

Сделана  большая доработка эмулятора в плане произвольного расширения
количества  выводимых на дисплей строк файлов,  а так же долгожданная 
функция  записи!  Автор  этих доработок  Evgeny Ugryumov. 

В Config.h можно настроить вывод на 4 вида дисплеев:
> OLED 128x64
> OLED 128x32
> LCD 1602
> LCD 2004
Так же добавлена возможность разворота изображения на 180 градусов. 
Автор "прикрученных" дисплеев - Rahost

Отдельное спасибо Alex Rauch за бета тестирование и помощи в поимке
глюков, Сергею MadCat за поддержку.

Статья о сборке эмулятора:
https://rahost-studio.ru/2024/03/fdd-zx-spectrum/
Дополнение:
https://rahost-studio.ru/2024/03/zx-fdd-2/
Возможные обновления данного кода будут в этой статейке:
https://rahost-studio.ru/2024/05/write-zx-fdd-emul/


RahostStudio - 08.06.2024
*/
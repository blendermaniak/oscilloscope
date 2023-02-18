# oscilloscope

Prototyp oscyloskopu 
Utworzono procedurę obsługi przerwania od ADC, w której punkty zmierzonego przebiegu przekopiowano do bufora. Po przekopiowaniu określonej liczby punktów
ustawiona zostaje flaga buff_flag, dzięki której wychodzimy z procedury obsługi przerwania do pętli while(1). W tejże pętli punkty są "rysowane" na wyświetlaczu
OLED. Po "narysowaniu" przebiegu flaga buff_flag jest zerowana, dzięki czemu możemy używać przerwania od ADC.

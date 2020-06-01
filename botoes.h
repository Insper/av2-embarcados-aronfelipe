#ifndef BOTOES_H
#define BOTOES_H

typedef struct {
  uint32_t width;     // largura (px)
  uint32_t height;    // altura  (px)
  uint32_t colorOn;   // cor do botão acionado
  uint32_t colorOff;  // cor do botão desligado
  uint32_t x;         // posicao x 
  uint32_t y;         // posicao y
  uint8_t status;
} t_but;

#define BUT_SIZE 100
#define BUT_SPACE 3

t_but but1 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 1*BUT_SPACE + BUT_SIZE/2, .y = BUT_SPACE+ BUT_SIZE/2, .status = 0};
t_but but2 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 2*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE, .y = BUT_SPACE + BUT_SIZE/2,
              .status = 0};
t_but but3 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 3*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE*2, .y = BUT_SPACE + BUT_SIZE/2,
              .status = 0};

t_but but4 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 1*BUT_SPACE + BUT_SIZE/2, .y = 2*BUT_SPACE + BUT_SIZE + BUT_SIZE/2,
              .status = 0};
t_but but5 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 2*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE, .y = 2*BUT_SPACE + BUT_SIZE + BUT_SIZE/2,
              .status = 0};
t_but but6 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 3*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE*2, .y = 2*BUT_SPACE + BUT_SIZE + BUT_SIZE/2,
              .status = 0};

t_but but7 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 1*BUT_SPACE + BUT_SIZE/2, .y = 3*BUT_SPACE + 2*BUT_SIZE + BUT_SIZE/2,
              .status = 0};
t_but but8 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 2*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE, .y = 3*BUT_SPACE + 2*BUT_SIZE + BUT_SIZE/2,
              .status = 0};
t_but but9 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 3*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE*2, .y = 3*BUT_SPACE + 2*BUT_SIZE + BUT_SIZE/2,
              .status = 0};

t_but but0 = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 2*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE, .y = 4*BUT_SPACE + 3*BUT_SIZE + BUT_SIZE/2,
              .status = 0};
t_but butx = {.width = BUT_SIZE, .height = BUT_SIZE,
              .colorOn = COLOR_GRAY, .colorOff = COLOR_GRAY,
              .x = 3*BUT_SPACE + BUT_SIZE/2 + BUT_SIZE*2, .y = 4*BUT_SPACE + 3*BUT_SIZE + BUT_SIZE/2,
              .status = 0};



#endif

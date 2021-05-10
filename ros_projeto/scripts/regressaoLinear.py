import numpy as np
import math
import cv2
import statsmodels.api as sm
from sklearn.linear_model import LinearRegression


def morpho_limpa(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )    
    return mask


def multiplot(imgs, legenda="No sub"):
    """ Função """
    fig, axes = plt.subplots(1,len(imgs), figsize=(24,8))    
    fig.suptitle(legenda)
    if len(imgs)==1: # Peculiaridade do subplot. Não é relevante para a questão
        ax = axes
        ax.imshow(cv2.cvtColor(imgs[0], cv2.COLOR_BGR2RGB))
        return
    for i in range(len(imgs)):
        axes[i].imshow(cv2.cvtColor(imgs[i], cv2.COLOR_BGR2RGB))
        
def multiplot_gray(imgs, legenda):
    """ Função que plota n imagens grayscale em linha"""
    fig, axes = plt.subplots(1,len(imgs), figsize=(26,8))    
    fig.suptitle(legenda)
    if len(imgs)==1: # Peculiaridade do subplot. Não é relevante para a questão
        ax = axes
        ax.imshow(imgs[0],  vmin=0, vmax=255, cmap="Greys_r")
        return
    for i in range(len(imgs)):
        axes[i].imshow(imgs[i], vmin=0, vmax=255, cmap="Greys_r")
        
        
# Função centro de massa baseada na aula 02  https://github.com/Insper/robot202/blob/master/aula02/aula02_Exemplos_Adicionais.ipynb
# Esta função calcula centro de massa de máscara binária 0-255 também, não só de contorno
def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    # Usando a expressão do centróide definida em: https://en.wikipedia.org/wiki/Image_moment
    if M["m00"] == 0 or M["m01"] == 0:
        return[0,0]
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,3)
    cv2.line(img,(x,y - size),(x, y + size),color,3)
    
def segmenta_linha_amarela_bgr(bgr):
    """ REturns a mask within the range"""

    low_yellow = (20, 50, 50)
    high_yellow = (45, 255, 255)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low_yellow, high_yellow)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )
    return mask    
 
def ajuste_linear_x_fy(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """ 
    pontos = np.where(mask==255)
    if np.any(pontos):
        ximg = pontos[1]
        yimg = pontos[0]
        yimg_c = sm.add_constant(yimg)
        model = sm.OLS(ximg,yimg_c)
        results = model.fit()
        coef_angular = results.params[1] # Pegamos o beta 1
        coef_linear =  results.params[0] # Pegamso o beta 0
        return coef_angular, coef_linear
    return 1, 1
def maskYellowBloqueiaDireita(mask):
    mask = cv2.rectangle(mask, (450,0), (720,480), (0,0,0), -1)
    return mask
    
def maskYellowBloqueiaEsquerda(mask):
    mask = cv2.rectangle(mask, (0,0), (200,480), (0,0,0), -1)
    return mask

def maskYellowBloqueiaEsquerdaMaior(mask):
    mask = cv2.rectangle(mask, (0,0), (300,480), (0,0,0), -1)
    return mask


def ajuste_linear(mask):
    """Recebe uma imagem já limiarizada e faz um ajuste linear
        retorna coeficientes linear e angular da reta
        e equação é da forma
        y = coef_angular*x + coef_linear
    """ 
    pontos = np.where(mask==255)
    ximg = pontos[1]
    yimg = pontos[0]
    yimg_c = sm.add_constant(yimg)
    ximg_c = sm.add_constant(ximg)
    model = sm.OLS(yimg,ximg_c)
    results = model.fit()
    coef_angular = results.params[1] # Pegamos o beta 1
    coef_linear =  results.params[0] # Pegamso o beta 0
    return coef_angular, coef_linear

def ajuste_linear_grafico_x_fy(mask):
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem"""
    coef_angular, coef_linear = ajuste_linear_x_fy(mask)
    print("x = {:3f}*y + {:3f}".format(coef_angular, coef_linear))
    pontos = np.where(mask==255) # esta linha é pesada e ficou redundante
    if np.any(pontos):
        ximg = pontos[1]
        yimg = pontos[0]
        y_bounds = np.array([min(yimg), max(yimg)])
        x_bounds = coef_angular*y_bounds + coef_linear
        print("x bounds", x_bounds)
        print("y bounds", y_bounds)
        x_int = x_bounds.astype(dtype=np.int64)
        y_int = y_bounds.astype(dtype=np.int64)
        mask_rgb =  cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.line(mask_rgb, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=11);    
        return mask_rgb, coef_angular, x_int
    return mask, coef_angular, [0,0]
            
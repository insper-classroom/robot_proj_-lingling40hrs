import numpy as np
import cv2
import statsmodels.api as sm
from sklearn.linear_model import LinearRegression

def morpho_limpa(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( mask, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )    
    return mask
    
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

def ajuste_linear_grafico_x_fy(mask):
    """Faz um ajuste linear e devolve uma imagem rgb com aquele ajuste desenhado sobre uma imagem"""
    coef_angular, coef_linear = ajuste_linear_x_fy(mask)
    #print("x = {:3f}*y + {:3f}".format(coef_angular, coef_linear))
    pontos = np.where(mask==255) # esta linha é pesada e ficou redundante
    if np.any(pontos):
        ximg = pontos[1]
        yimg = pontos[0]
        y_bounds = np.array([min(yimg), max(yimg)])
        x_bounds = coef_angular*y_bounds + coef_linear
        #print("x bounds", x_bounds)
        #print("y bounds", y_bounds)
        x_int = x_bounds.astype(dtype=np.int64)
        y_int = y_bounds.astype(dtype=np.int64)
        mask_rgb =  cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.line(mask_rgb, (x_int[0], y_int[0]), (x_int[1], y_int[1]), color=(0,0,255), thickness=11);    
        return mask_rgb, coef_angular, x_int
    return mask, coef_angular, [0,0]

#---------------Mascaras aplicadas para fazer o robo tender a um lado
def maskYellowBloqueiaDireita(mask):
    mask = cv2.rectangle(mask, (450,0), (720,480), (0,0,0), -1)
    return mask
    
def maskYellowBloqueiaEsquerda(mask):
    mask = cv2.rectangle(mask, (0,0), (200,480), (0,0,0), -1)
    return mask

def maskYellowBloqueiaEsquerdaMaior(mask):
    mask = cv2.rectangle(mask, (0,0), (300,480), (0,0,0), -1)
    return mask
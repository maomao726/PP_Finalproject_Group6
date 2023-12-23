import cv2
import numpy as np
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--image", "-i", help="image name", type=str, required=True)
parser.add_argument("--coord", "-c", help="image coord with respect to 4 court corners", type=str, required=True)
args = parser.parse_args()

def convolution(img, kernel, isGray):
    kernelSize = np.shape(kernel)[0]
    result = np.zeros(img.shape)
    if isGray:
        img = np.pad(img, (int(kernelSize / 2), int(kernelSize / 2)),mode="edge")
    imgSize = np.shape(img)
    if not isGray:           ##RGB
        imgR, imgG, imgB = img[:, :, 0], img[:, :, 1], img[:, :, 2]
        sizeX, sizeY,_ = imgSize
        x = sizeX-kernelSize+1
        y = sizeY-kernelSize+1
        for i in range(x):
            for j in range(y):
                newR = np.sum(np.multiply(kernel , imgR[i:i+kernelSize, j:j+kernelSize]))
                newG = np.sum(np.multiply(kernel , imgG[i:i+kernelSize, j:j+kernelSize]))
                newB = np.sum(np.multiply(kernel , imgB[i:i+kernelSize, j:j+kernelSize]))
                result[i][j] = [newR, newG, newB]
    else:                       ##GRAY
        sizeX, sizeY = imgSize
        x = sizeX-kernelSize+1
        y = sizeY-kernelSize+1
        for i in range(x):
            for j in range(y):
                result[i][j] = np.sum(np.multiply(kernel , img[i:i+kernelSize, j:j+kernelSize]))
    return result

def outline_searching(img_gray, mask):

    in_image = cv2.polylines(np.zeros(img_gray.shape, dtype=np.uint8), [mask], True, 255)
    ## cv2.imshow("w_l", in_image)
  

    ## gradient magnitude
    v_filter = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]], np.float32)
    h_filter = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], np.float32)
    v_grad = convolution(img_gray, v_filter, isGray=True)
    h_grad = convolution(img_gray, h_filter, isGray=True)
    grad_mag = np.sqrt(np.square(v_grad) + np.square(h_grad))
    grad_mag = (grad_mag * 255 / grad_mag.max()).astype('uint8')
    _, grad_mag_bin = cv2.threshold(grad_mag, 80, 255, cv2.THRESH_BINARY)

    ## cv2.imshow("w", grad_mag_bin)


    # lines = cv2.HoughLinesP(grad_mag_bin, 1, np.pi/180, 150, minLineLength=20, maxLineGap=50)
    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     print(x1, y1)
    #     cv2.line(grad_mag_bin, (x1, y1), (x2, y2), 255, 1)


    # line_image = cv2.bitwise_and(roi, (in_image + grad_mag_bin))
    
    # lines = cv2.HoughLinesP(grad_mag_bin, 1, np.pi/180, 150, minLineLength=5, maxLineGap=50)

    # # for idx in range(lines_pt.shape[0]-1):
    # #     p1 = lines_pt[idx]
    # #     p2 = lines_pt[idx+1]

    # #     ## line
    # #     line = cv2.line(line_image, p1, p2, 255, 1)

    # for line in lines:
    #     x1, y1, x2, y2 = line[0]
    #     print(x1, y1)
    #     cv2.line(grad_mag_bin, (x1, y1), (x2, y2), 255, 1)

    line_image = in_image + grad_mag_bin
    ## cv2.imshow("w_l_g", line_image)


    in_image = cv2.fillPoly(np.zeros(img_gray.shape, dtype=np.uint8), [mask], 255)
    line_image = in_image + grad_mag_bin
    ## cv2.imshow("w_l_g", line_image)
    line_image = in_image + grad_mag_bin
    # for line in lines:
    #     ## get line's vector
    #     x1, y1, x2, y2 = line[0]
    #     direction = np.array([(x2 - x1) , (y2 - y1)])
    #     distance = np.sqrt((y2 - y1)**2 + (x2 - x1) ** 2)
    #     direction = direction / distance
    contours, _ = cv2.findContours(line_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    max_contour = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            e = 0.0001 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, e, True)
            max_contour = approx
       
    contour_img = cv2.fillPoly(np.zeros(line_image.shape, np.uint8), [max_contour], 255)

    ## cv2.imshow("w_c", contour_img)

    return contour_img
    # lines_pt = np.concatenate([mask, mask[0].reshape((1, -1))], axis = 0)
    # for idx in range(lines_pt.shape[0]-1):
    #     p1 = lines_pt[idx]
    #     p2 = lines_pt[idx+1]

    #     ## get line's vector
    #     direction = np.array([(p2[0] - p1[0]) , (p2[1] - p1[1])])
    #     distance = np.sqrt((p2[1] - p1[1])**2 + (p2[0] - p1[0]) ** 2)
    #     direction = direction / distance

    #     ## line
    #     line = cv2.line(np.zeros((img_height, img_width), dtype=np.uint8), p1, p2, 255, 2)
    #     search_list = np.argwhere(line==255).tolist()

    #     ## dilate line to fit the area
        
    #     for pix_head in search_list:
    #         num_pixel = 0
    #         search_stack = []
    #         if pix_head[0] > 0:
    #             search_stack.append([pix_head[0]-1, pix_head[1]])
    #         if pix_head[0] < img_height-1:
    #             search_stack.append([pix_head[0]+1, pix_head[1]])
    #         if pix_head[1] > 0:
    #             search_stack.append([pix_head[0], pix_head[1]-1])
    #         if pix_head[1] < img_width-1:
    #             search_stack.append([pix_head[0], pix_head[1]+1])
    #         line_image_cpy = np.copy(line_image)
            
    #         while len(search_stack) != 0:
    #             pix = search_stack.pop()
    #             if line_image_cpy[pix[0], pix[1]] != 255:
    #                 num_pixel += 1
    #                 line_image_cpy[pix[0], pix[1]] = 255
    #                 if num_pixel > 0.01 * img_height * img_width:
    #                     break
    #                 if pix[0] > 0:
    #                     search_stack.append([pix[0]-1, pix[1]])
    #                 if pix[0] < img_height-1:
    #                     search_stack.append([pix[0]+1, pix[1]])
    #                 if pix[1] > 0:
    #                     search_stack.append([pix[0], pix[1]-1])
    #                 if pix[1] < img_width-1:
    #                     search_stack.append([pix[0], pix[1]+1])
                    
    #         if num_pixel < 0.01 * img_height * img_width:
    #             line_image = np.copy(line_image_cpy)
    #             ## cv2.imshow("w", line_image)
    #             cv2.waitKey(1)
        
    #     # while search_stack.shape[0] != 0:
    #     #     line_px = search_stack[0]
    #     #     search_stack = search_stack[1:]
    # contours, _ = cv2.findContours(line_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # max_area = 0
    # max_contour = None
    # for contour in contours:
    #     print(contour)
    #     area = cv2.contourArea(contour)
    #     if area > max_area:
    #         max_area = area
    #         max_contour = contour
        

    # ## cv2.imshow("w", line_image)
    # cv2.waitKey(0)

if __name__ == "__main__":
    print(args)


    corner = []
    img = cv2.imread(args.image)

    ## cv2.imshow("origin", img)

    # with open(args.coord, "r") as f:
    #     for line in f:
    #         corner.append((float(line.split(";")[0]), float(line.split(";")[1])))
    #     f.close()
    corner = eval(args.coord)
    
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (3, 3), 1)

    mask = np.array(corner, dtype=int)
    mask = np.concatenate([mask, mask[0].reshape((1, -1))], axis = 0)

    ## original result
    img_cpy = np.copy(img)
    shape = np.full(img.shape, (0, 0, 255), dtype=np.uint8)
    shape = cv2.fillPoly(shape, [mask], (0, 255, 0))
    img_cpy = cv2.addWeighted(img_cpy, 0.7, shape, 0.3, 0)
    ## cv2.imshow("origin_mask", shape)

    ## cv2.imshow("origin_res", img_cpy)


    in_mask = outline_searching(img_gray, mask)

    ## After Combining
    img_cpy = np.copy(img)
    shape = np.full(img.shape, (0, 0, 255), dtype=np.uint8)
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if in_mask[i, j] == 255:
                shape[i, j] = [0, 255, 0]
    img = cv2.addWeighted(img, 0.7, shape, 0.3, 0)
    ## cv2.imshow("new_mask", shape)

    cv2.imshow("new_res", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    

    

    

# """Project 04 - Advanced Lane Detection
# Usage:
#   project04.py <input_video> <output_video> [-c <camera_file>]
#   project04.py (-h | --help)
# Options:
#   -h --help         Show this screen.
#   -c <camera_file>  Specify camera calibration file [default: camera_data.npz]
# """
# import os
# import cv2
# import glob
# import numpy as np
# from math import *
# import matplotlib.pyplot as plt
# import matplotlib.image as mpimage
# import collections
# import imageio
# imageio.plugins.ffmpeg.download()
# from itertools import chain
# from functools import reduce
# from scipy.signal import find_peaks_cwt
# from moviepy.editor import VideoFileClip
#
#

# def calibrate_camera(cal_images, nx, ny):
#     objpoints = []  # 3D points
#     imgpoints = []  # 2D points
#
#     objp = np.zeros((nx*ny,3), np.float32)
#     objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1, 2)
#
#     for fname in cal_images:
#         img = cv2.imread(fname)
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#         ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)
#         if ret == True:
#             objpoints.append(objp)
#             imgpoints.append(corners)
#
#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
#
#     return mtx, dist
#
# def camera_setup(calibration_path):
#     cal_images = glob.glob(calibration_path)
#     nx, ny = 9, 6
#     cam_mtx, cam_dist = calibrate_camera(cal_images, nx, ny)
#     return cam_mtx, cam_dist
#
# def get_perspective_transform(image, src_in = None, dst_in = None, display=False):
#     img_size = image.shape
#     if src_in is None:
#         src = np.array([[585. /1280.*img_size[1], 455./720.*img_size[0]],
#                         [705. /1280.*img_size[1], 455./720.*img_size[0]],
#                         [1130./1280.*img_size[1], 720./720.*img_size[0]],
#                         [190. /1280.*img_size[1], 720./720.*img_size[0]]], np.float32)
#     else:
#         src = src_in
#
#     if dst_in is None:
#         dst = np.array([[300. /1280.*img_size[1], 100./720.*img_size[0]],
#                         [1000./1280.*img_size[1], 100./720.*img_size[0]],
#                         [1000./1280.*img_size[1], 720./720.*img_size[0]],
#                         [300. /1280.*img_size[1], 720./720.*img_size[0]]], np.float32)
#     else:
#         dst = dst_in
#
#     warp_m = cv2.getPerspectiveTransform(src, dst)
#     warp_minv = cv2.getPerspectiveTransform(dst, src)
#
#     if display:
#         plt.subplot(1,2,1)
#         plt.hold(True)
#         plt.imshow(image, cmap='gray')
#         colors = ['r+','g+','b+','w+']
#         for i in range(4):
#             plt.plot(src[i,0],src[i,1],colors[i])
#
#         im2 = cv2.warpPerspective(image, warp_m, (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR)
#         plt.subplot(1,2,2)
#         plt.hold(True)
#         plt.imshow(im2, cmap='gray')
#         for i in range(4):
#             plt.plot(dst[i,0],dst[i,1],colors[i])
#         plt.show()
#     return warp_m, warp_minv
#
# def find_perspective_points(image):
#     edges = find_edges(image)
#
#     # Computing perspective points automatically
#     rho = 2              # distance resolution in pixels of the Hough grid
#     theta = 1*np.pi/180  # angular resolution in radians of the Hough grid
#     threshold = 100       # minimum number of votes (intersections in Hough grid cell)
#     min_line_length = 100 # minimum number of pixels making up a line
#     max_line_gap = 25    # maximum gap in pixels between connectable line segments
#
#     angle_min_mag = 20*pi/180
#     angle_max_mag = 65*pi/180
#
#     lane_markers_x = [[], []]
#     lane_markers_y = [[], []]
#
#     masked_edges = np.copy(edges)
#     masked_edges[:edges.shape[0]*6//10,:] = 0
#     lines = cv2.HoughLinesP(masked_edges, rho, theta, threshold, min_line_length, max_line_gap)
#     for line in lines:
#         for x1,y1,x2,y2 in line:
#             theta = atan2(y1-y2, x2-x1)
#             rho = ((x1+x2)*cos(theta) + (y1+y2)*sin(theta))/2
#             if (abs(theta) >= angle_min_mag and abs(theta) <= angle_max_mag):
#                 if theta > 0: # positive theta is downward in image space?
#                     i = 0 # Left lane marker
#                 else:
#                     i = 1 # Right lane marker
#                 lane_markers_x[i].append(x1)
#                 lane_markers_x[i].append(x2)
#                 lane_markers_y[i].append(y1)
#                 lane_markers_y[i].append(y2)
#
#     if len(lane_markers_x[0]) < 1 or len(lane_markers_x[1]) < 1:
#         # Failed to find two lane markers
#         return None
#
#     p_left  = np.polyfit(lane_markers_y[0], lane_markers_x[0], 1)
#     p_right = np.polyfit(lane_markers_y[1], lane_markers_x[1], 1)
#
#     # Find intersection of the two lines
#     apex_pt = np.linalg.solve([[p_left[0], -1], [p_right[0], -1]], [-p_left[1], -p_right[1]])
#     top_y = ceil(apex_pt[0] + 0.075*edges.shape[0])
#
#     bl_pt = ceil(np.polyval(p_left, edges.shape[0]))
#     tl_pt = ceil(np.polyval(p_left, top_y))
#
#     br_pt = ceil(np.polyval(p_right, edges.shape[0]))
#     tr_pt = ceil(np.polyval(p_right, top_y))
#
#     src = np.array([[tl_pt, top_y],
#                     [tr_pt, top_y],
#                     [br_pt, edges.shape[0]],
#                     [bl_pt, edges.shape[0]]], np.float32)
#
#     get_perspective_transform(edges, src_in = src, dst_in = None, display=False)
#     return src
#
# def find_edges(image, mask_half=False):
#     hls = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_RGB2HLS)
#     s = hls[:,:,2]
#     gray = (0.5*image[:,:,0] + 0.4*image[:,:,1] + 0.1*image[:,:,2]).astype(np.uint8)
#
#     _, gray_binary = cv2.threshold(gray.astype('uint8'), 130, 255, cv2.THRESH_BINARY)
#
#     # switch to gray image for laplacian if 's' doesn't give enough details
#     total_px = image.shape[0]*image.shape[1]
#     laplacian = cv2.Laplacian(gray, cv2.CV_32F, ksize=21)
#     mask_one = (laplacian < 0.15*np.min(laplacian)).astype(np.uint8)
#     if cv2.countNonZero(mask_one)/total_px < 0.01:
#         laplacian = cv2.Laplacian(gray, cv2.CV_32F, ksize=21)
#         mask_one = (laplacian < 0.075*np.min(laplacian)).astype(np.uint8)
#
#     _, s_binary = cv2.threshold(s.astype('uint8'), 150, 255, cv2.THRESH_BINARY)
#     mask_two = s_binary
#
#
#     combined_binary = np.clip(cv2.bitwise_and(gray_binary,
#                         cv2.bitwise_or(mask_one, mask_two)), 0, 1).astype('uint8')
#
#     return combined_binary
#
# ym_per_pix = 30/720 # meters per pixel in y dimension
# xm_per_pix = 3.7/700 # meteres per pixel in x dimension
#
# class Lane():
#     def __init__(self, base_pt, img_size, cache_length):
#         # was the line detected in the last iteration?
#         self.detected = False
#         # x values of the last n fits of the line
#         self.recent_xfitted = collections.deque(maxlen=cache_length)
#         self.recent_yfitted = collections.deque(maxlen=cache_length)
#
#         #polynomial coefficients for the most recent fit
#         self.current_fit = [np.array([False])]
#         #radius of curvature of the line in some units
#         self.radius_of_curvature = None
#         #distance in meters of vehicle center from the line
#         self.insanity = 0.0
#
#         self.current_xfit = None
#
#         self.img_size = img_size
#         self.base_pt = base_pt
#
#         self.yvals = np.linspace(0, img_size[0], 101)
#         self.mask = np.ones(img_size, dtype=np.uint8)*255
#
#         self.dropped_frames = 0
#
#     def add_lane_pixels(self, x, y):
#         """
#         Adds lane pixels and recomputes curve-fit.
#         """
#         # Use all pixels from previous detections for curve fit
#         weights = np.ones(len(self.recent_xfitted))
#         if len(weights) > 1:
#             weights[0] = 0.8
#             weights[1:] = 0.2/(len(weights) - 1)
#
#             w_x = reduce(lambda a,b: a + b[0]*b[1], zip(weights, self.recent_xfitted), np.zeros(len(self.yvals)))
#             w_y = reduce(lambda a,b: a + b[0]*b[1], zip(weights, self.recent_yfitted), np.zeros(len(self.yvals)))
#         else:
#             w_x, w_y = [], []
#         x_hist = np.fromiter(chain(w_x, x), np.int32)
#         y_hist = np.fromiter(chain(w_y, y), np.int32)
#
#         try:
#             p_lane = np.polyfit(y_hist, x_hist, 2)
#             rad_curv = self.compute_rad_curv(x_hist, y_hist)
#             self.detected = self.sanity_check_lane(rad_curv)
#         except Exception as e:
#             print(e)
#             self.detected = False
#
#         if self.detected and len(p_lane) == 3:
#             x_fit = p_lane[0]*self.yvals**2 + p_lane[1]*self.yvals + p_lane[2]
#
#             self.current_xfit = x_fit   # For drawing
#
#             self.recent_xfitted.append(x_fit)
#             self.recent_yfitted.append(self.yvals)
#
#             self.radius_of_curvature = rad_curv
#             self.current_fit = p_lane
#             self.dropped_frames = 0
#         else:
#             # Sanity check failed
#             # Use last fit if current one failed
#             p_lane = self.current_fit
#             rad_curv = self.radius_of_curvature
#             x_fit = p_lane[0]*self.yvals**2 + p_lane[1]*self.yvals + p_lane[2]
#             self.dropped_frames += 1
#
#         # Update ROI mask
#         self.mask.fill(0)
#         # http://stackoverflow.com/a/35902430/538379
#         pts = np.transpose(np.vstack([x_fit, self.yvals])).reshape((-1,1,2)).astype(np.int32)
#         cv2.drawContours(self.mask, pts, -1, (255,255,255), thickness=80)
#
#
#     @staticmethod
#     def compute_rad_curv(xvals, yvals):
#         fit_cr = np.polyfit(yvals*ym_per_pix, xvals*xm_per_pix, 2)
#         y_eval = np.max(yvals)
#         curverad = ((1 + (2*fit_cr[0]*y_eval + fit_cr[1])**2)**1.5) \
#                                      /np.absolute(2*fit_cr[0])
#         return curverad
#
#
#     def sanity_check_lane(self, R):
#         """
#         Checks new radius of curvature `R` against the radius stored in the object.
#         """
#         # Return true if there is no prior data
#         if self.radius_of_curvature is None:
#             return True
#
#         R0 = self.radius_of_curvature
#         self.insanity = abs(R-R0)/R0
#         return self.insanity <= 0.5  # Max change from frame to frame is 200%
#
#
#     def detect_from_mask(self, image):
#         mask_lanes = cv2.bitwise_and(image, self.mask)
#         all_pts = cv2.findNonZero(mask_lanes)
#         if all_pts is not None:
#             all_pts = all_pts.reshape((-1,2))
#             self.add_lane_pixels(all_pts[:,0], all_pts[:,1])
#         else:
#             self.detected = False
#
#     def draw_lane(self, image):
#         """
#         Draws lane on given image
#         """
#         pts = np.array([np.transpose(np.vstack([self.current_xfit, self.yvals]))])
#         cv2.fillPoly(image, np.int_([pts]), (0,255, 0))
#         return image
#
# def reject_outliers(x_list, y_list):
#     if not x_list or not y_list:
#         return x_list, y_list
#     mu_x, mu_y = np.mean(x_list), np.mean(y_list)
#     sig_x, sig_y = np.std(x_list), np.std(y_list)
#     new_x, new_y = zip(*[(x, y) for (x,y) in zip(x_list, y_list)
#                                  if abs(x - mu_x) < 2*sig_x and abs(y - mu_y) < 2*sig_y])
#     return new_x, new_y
#
# def sliding_window(image, left_lane, right_lane, base_pts, num_bands = 10, window_width = 0.2):
#     """Uses histogram and sliding window to detect lanes from scratch"""
#
#     height = image.shape[0]
#     band_height = int(1./num_bands * height)   # Divide image into horizontal bands
#     band_width = int(window_width*image.shape[1])
#
#     l_x, l_y, r_x, r_y = [], [], [], []
#
#     base_left, base_right = base_pts
#
#     idx_left, idx_right = base_pts
#     for i in reversed(range(num_bands)):
#         w_left = image[i*band_height:(i+1)*band_height,base_left-band_width//2:base_left+band_width//2]
#         w_right = image[i*band_height:(i+1)*band_height,base_right-band_width//2:base_right+band_width//2]
#
#         left_y_pt, left_x_pt = np.nonzero(w_left)
#         right_y_pt, right_x_pt = np.nonzero(w_right)
#
#         l_x.extend(left_x_pt + base_left-band_width//2)
#         l_y.extend(left_y_pt + i*band_height)
#         r_x.extend(right_x_pt+ base_right-band_width//2)
#         r_y.extend(right_y_pt+ i*band_height)
#
#         # Find 'x' with maximum nonzero elements as baseline for next window
#         s_left = np.sum(w_left, axis=0)
#         s_right = np.sum(w_right, axis=0)
#         if np.any(s_left > 0):
#             base_left = np.argmax(s_left) + base_left-band_width//2
#         if np.any(s_right > 0):
#             base_right = np.argmax(s_right) + base_right-band_width//2
#
#     l_x, l_y = reject_outliers(l_x, l_y)
#     r_x, r_y = reject_outliers(r_x, r_y)
#
#     left_lane.add_lane_pixels(l_x, l_y)
#     right_lane.add_lane_pixels(r_x, r_y)
#
#     return left_lane, right_lane
#
# def histogram_base_points(lanes, min_peak = 25.0):
#     """Uses histogram to find possible base points for lane lines"""
#     hist = np.sum(lanes[int(lanes.shape[0]*0.5):,:], axis=0)
#
#     widths = [100]
#     idx = find_peaks_cwt(hist, widths, max_distances=widths, noise_perc=50)
#     if len(idx) < 2:
#         return None
#
#     # Avoid edges
#     idx = [i for i in idx if i > lanes.shape[1]*0.1
#                              and i < lanes.shape[1]*0.9
#                              and max(hist[i-50:i+50]) > min_peak]
#
#     return [min(idx), max(idx)]
#
# def process_image(image, key_frame_interval=20, cache_length=10):
#     global cam_mtx, cam_dist
#
#     if process_image.cache is None:
#
#         left_lane = Lane(int(0.16*image.shape[0]), image.shape[:2], cache_length=cache_length)
#         right_lane = Lane(int(0.62*image.shape[0]), image.shape[:2], cache_length=cache_length)
#
#         cache = {'cam_mtx': cam_mtx,
#                  'cam_dist': cam_dist,
#                  'warp_m': None,
#                  'warp_minv': None,
#                  'frame_ctr': 0,
#                  'left': left_lane,
#                  'right': right_lane,
#                  'base_pts': None}
#     else:
#         cache = process_image.cache
#
#
#     left_lane = cache['left']
#     right_lane = cache['right']
#
#     # Preprocess image and find edges using thresholding
#     undist = cv2.undistort(image, cam_mtx, cam_dist, None, cam_mtx)
#
#     if cache['warp_m'] is None:# or cache['frame_ctr'] % key_frame_interval == 0:
#         src = find_perspective_points(undist)
#         warp_m, warp_minv = get_perspective_transform(image, src_in = src)
#
#         if src is not None:
#             # Save only if customized perspective transform is found
#             cache['warp_m'] = warp_m
#             cache['warp_minv'] = warp_minv
#     else:
#         warp_m, warp_minv = cache['warp_m'], cache['warp_minv']
#
#     edges = find_edges(undist)
#     warp_edges = cv2.warpPerspective(edges, warp_m, (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR)
#
#     # Reverse pipeline (warp before thresholding)
#     # warp_img = cv2.warpPerspective(undist, warp_m, (image.shape[1], image.shape[0]), flags=cv2.INTER_LINEAR)
#     # warp_edges = find_edges(warp_img)
#
#     base_pts = cache['base_pts']
#     if base_pts is None: #or cache['frame_ctr'] % key_frame_interval == 0:
#         new_base_pts = histogram_base_points(warp_edges)
#
#         if new_base_pts is not None:
#             base_pts = new_base_pts
#         else:
#             # Could not find new base points
#             # Re-use previous data if base points could not be found
#             cache['frame_ctr'] = cache['frame_ctr'] - 1 # Make sure we try again in the next frame
#             return undist
#
#     if ((left_lane.current_xfit is None or left_lane.dropped_frames > 16)
#             or (right_lane.current_xfit is None or right_lane.dropped_frames > 16)):
#         # Detect from scratch
#         left_lane.radius_of_curvature = None
#         right_lane.radius_of_curvature = None
#         sliding_window(warp_edges, left_lane, right_lane, base_pts)
#     else:
#         left_lane.detect_from_mask(warp_edges)
#         right_lane.detect_from_mask(warp_edges)
#
#     cache['frame_ctr'] = cache['frame_ctr'] + 1
#     cache['base_pts'] = base_pts
#     process_image.cache = cache
#
#     # Create an image to draw the lines on
#     color_warp = np.zeros_like(image).astype(np.uint8)
#
#     yvals = left_lane.yvals
#     left_fitx = left_lane.current_xfit
#     right_fitx = right_lane.current_xfit
#
#     # Create an image to draw the lines on
#     color_warp = np.zeros_like(image).astype(np.uint8)
#
#     # Recast the x and y points into usable format for cv2.fillPoly()
#     pts_left = np.array([np.transpose(np.vstack([left_fitx, yvals]))])
#     pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, yvals])))])
#     pts = np.hstack((pts_left, pts_right))
#
#     # Draw the lane onto the warped blank image
#     cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
#
#     # Draw lane markers
#     pts = np.transpose(np.vstack([left_lane.current_xfit, left_lane.yvals])).reshape((-1,1,2)).astype(np.int32)
#     cv2.drawContours(color_warp, pts, -1, (255,0,0), thickness=30)
#     pts = np.transpose(np.vstack([right_lane.current_xfit, right_lane.yvals])).reshape((-1,1,2)).astype(np.int32)
#     cv2.drawContours(color_warp, pts, -1, (0,0,255), thickness=30)
#
#     # Warp the blank back to original image space using inverse perspective matrix (Minv)
#     newwarp = cv2.warpPerspective(color_warp, warp_minv, (image.shape[1], image.shape[0]))
#
#     # Combine the result with the original image
#     result = cv2.addWeighted(undist, 1, newwarp, 0.3, 0)
#
#     left_r = left_lane.radius_of_curvature
#     right_r = right_lane.radius_of_curvature
#     middle = (left_fitx[-1] + right_fitx[-1])//2
#     veh_pos = image.shape[1]//2
#
#     dx = (veh_pos - middle)*xm_per_pix # Positive if on right, Negative on left
#
#     font = cv2.FONT_HERSHEY_SIMPLEX
#     cv2.putText(result,'Left radius of curvature  = %.2f m'%(left_r),(50,50), font, 1,(255,255,255),2,cv2.LINE_AA)
#     cv2.putText(result,'Right radius of curvature = %.2f m'%(right_r),(50,80), font, 1,(255,255,255),2,cv2.LINE_AA)
#     cv2.putText(result,'Vehicle position : %.2f m %s of center'%(abs(dx), 'left' if dx < 0 else 'right'),(50,110),
#                         font, 1,(255,255,255),2,cv2.LINE_AA)
#
#     is_tracking = left_lane.detected or right_lane.detected
#     cv2.putText(result,'Tracking Locked' if is_tracking else 'Tracking Lost',(50,140),
#             font, 1,(0,255,0) if is_tracking else (255,0,0), 3,cv2.LINE_AA)
#
#     cache['left'] = left_lane
#     cache['right'] = right_lane
#
#     return result
#
# def clear_cache():
#     process_image.cache = None
#
# from docopt import docopt
#
# if __name__ == '__main__':
#     arguments = docopt(__doc__, version='Advanced Lane Lines 1.0')
#
#     clear_cache()
#     cam_file = arguments['-c']
#     #
#     # if not os.path.isfile(cam_file):
#     #     print('Calibrating camera ...')
#     #     #cam_mtx, cam_dist = camera_setup('camera_cal/calibration*.jpg')
#     #     #np.savez_compressed(cam_file, cam_mtx=cam_mtx, cam_dist=cam_dist)
#     # else:
#     #     print('Loading camera data from', cam_file)
#     #     data = np.load(cam_file)
#     #     cam_mtx = data['cam_mtx']
#     #     cam_dist = data['cam_dist']
#
#     in_file = arguments['<input_video>']
#     out_file = arguments['<output_video>']
#
#     print('Processing video ...')
#     clip2 = VideoFileClip(in_file)
#     vid_clip = process_image(clip2)
#     vid_clip.write_videofile(out_file, audio=False)



#####EXTRA CODE


        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #img = cv2.blur(frame,(5,5))
       # equ = cv2.equalizeHist(frame)
       #  (cnts, _) = cv2.findContours(frame.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
       #  cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
       #  screenCnt = None

        # for c in cnts:
        #     # approximate the contour
        #     peri = cv2.arcLength(c, True)
        #     approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        #     print "1"
       # cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)

        #lines = cv2.Canny(equ, 100, 150)
        #res = np.hstack((frame, equ))  # stacking images side-by-side
        # cv2.imshow('res.png', frame)
        # cv2.waitKey(0)



        #
        # edges = find_edges(frame)
        #
        #
        #
        #
        #
        # rho = 2  # distance resolution in pixels of the Hough grid
        # theta = 1 * np.pi / 180  # angular resolution in radians of the Hough grid
        # threshold = 100  # minimum number of votes (intersections in Hough grid cell)
        # min_line_length = 100  # minimum number of pixels making up a line
        # max_line_gap = 25  # maximum gap in pixels between connectable line segments
        #
        # angle_min_mag = 20 * np.pi / 180
        # angle_max_mag = 65 * np.pi / 180
        #
        # lane_markers_x = [[], []]
        # lane_markers_y = [[], []]
        # masked_edges = np.copy(edges)
        # masked_edges[:edges.shape[0] * 6 // 10, :] = 0
        # lines = cv2.HoughLinesP(edges, rho, theta, threshold, min_line_length, max_line_gap)
        #
        # for line in lines:
        #     for x1, y1, x2, y2 in line:
        #         theta = math.atan2(y1 - y2, x2 - x1)
        #         rho = ((x1 + x2) * math.cos(theta) + (y1 + y2) * math.sin(theta)) / 2
        #         if (abs(theta) >= angle_min_mag and abs(theta) <= angle_max_mag):
        #             if theta > 0:  # positive theta is downward in image space?
        #                 i = 0  # Left lane marker
        #             else:
        #                 i = 1  # Right lane marker
        #             lane_markers_x[i].append(x1)
        #             lane_markers_x[i].append(x2)
        #             lane_markers_y[i].append(y1)
        #             lane_markers_y[i].append(y2)
        #
        # if len(lane_markers_x[0]) < 1 or len(lane_markers_x[1]) < 1:
        #     # Failed to find two lane markers
        #     return None
        #
        # p_left = np.polyfit(lane_markers_y[0], lane_markers_x[0], 1)
        # p_right = np.polyfit(lane_markers_y[1], lane_markers_x[1], 1)
        #
        # # Find intersection of the two lines
        # apex_pt = np.linalg.solve([[p_left[0], -1], [p_right[0], -1]], [-p_left[1], -p_right[1]])
        # top_y = ceil(apex_pt[0] + 0.075 * edges.shape[0])
        #
        # bl_pt = ceil(np.polyval(p_left, edges.shape[0]))
        # tl_pt = ceil(np.polyval(p_left, top_y))
        #
        # br_pt = ceil(np.polyval(p_right, edges.shape[0]))
        # tr_pt = ceil(np.polyval(p_right, top_y))
        #
        # src = np.array([[tl_pt, top_y],
        #                 [tr_pt, top_y],
        #                 [br_pt, edges.shape[0]],
        #                 [bl_pt, edges.shape[0]]], np.float32)
        #
        # get_perspective_transform(edges, src_in=src, dst_in=None, display=False)
        # return src
        #
        #
        #








#
#
#
#         #frame = imutils.resize(frame,width=400)
#
#         #yellow - 30-37 25-40 40 and 55
#         lowerhdark = 130/2
#
#         lowervlight = 38*2.55
#
#         zHueLow = np.array([0, 0, 0], dtype="uint8")
#         zHueHigh = np.array([200/2, 15*2.55, 70*2.55], dtype="uint8")
#
#         upperhdark = 360/2
#         uppersdark = 18*2.55
#         uppervdark = 50*2.55
#
#         lowerdark = np.array([lowerhdark, 0, 0], dtype="uint8")
#         upperdark = np.array([upperhdark, uppersdark, uppervdark], dtype="uint8")
#
#         upperhlight = 47 / 2
#         upperslight = 25 * 2.55
#         uppervlight = 93 * 2.55
#
#         lowerlight = np.array([16/2, 13*2.55, lowervlight], dtype="uint8")
#         upperlight = np.array([upperhlight, upperslight, uppervlight], dtype="uint8")
#
#         loweryellow = np.array([17/2, 14*2.55, 32*2.55], dtype="uint8")
#         upperyellow = np.array([43/2, 46*2.55, 85*2.55], dtype="uint8")
#
#         lowerwhite = np.array([0, 0*2.55, 45*2.55], dtype="uint8")
#         upperwhite = np.array([300/2, 7*2.55, 55*2.55], dtype="uint8")
#
# g = cv2.bitwise_not(g)
# can = cannyG(gray)
# lines = cv2.HoughLinesP(can, 1, np.pi / 180, threshold, minLineLength, maxLineGap)
# lines = cv2.HoughLines(can, 1, np.pi / 180, threshold, minLineLength, maxLineGap)
# for x1, y1, x2, y2 in lines[0]:
#     cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
# g = cannyG(g)
# g = cv2.bitwise_not(g)



        #         hsv1, hsv2, hsv3 = cv2.split(hsv)
#         hsl1,hsl2,hsl3 = cv2.split(hls)
#         hsvt = hsv
#         #@#@$$!#rd = image_wrap(hsv, np.zeros(hsv.shape), hsv.shape)
#
#         #edges = canny(gray)
#         hsv = cv2.medianBlur(hsv, 9)
#
#         #hsv = gaussian_blur(hsv, 7)
#         yellowMask = cv2.inRange(hsv, loweryellow, upperyellow)
#         roadMask1 = cv2.inRange(hsv, lowerdark, upperdark)
#         roadMask2 = cv2.inRange(hsv, lowerlight, upperlight)
#         whiteMask = cv2.inRange(hsv, lowerwhite, upperwhite)
#         roadMask3 = cv2.inRange(hsv, zHueLow, zHueHigh)
#

        #smask = cv2.erode(smask, (5, 5), iterations=7)
        #smask = cv2.dilate(smask, (5, 5), iterations=7)
        # line = cv2.bitwise_and(frame, frame, mask=smask)
        # g = cv2.cvtColor(line, cv2.COLOR_HSV2BGR)
        # g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
        # can = cannyG(gray)
        #
        # threshold = 40  # minimum number of votes (intersections in Hough grid cell)
        # minLineLength = 100
        # maxLineGap = 10
        # # lines1 = cv2.HoughLines(can, 1, np.pi / 180, threshold, minLineLength, maxLineGap)
        # # if lines1 is not None:
        # #     for rho, theta in lines1[0]:
        # #         print np.pi/180*10
        # #         a = np.cos(theta)
        # #         b = np.sin(theta)
        # #         x0 = a * rho
        # #         y0 = b * rho
        # #         x1 = int(x0 + 1000 * (-b))
        # #         y1 = int(y0 + 1000 * (a))
        # #         x2 = int(x0 - 1000 * (-b))
        # #         y2 = int(y0 - 1000 * (a))
        # #         cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        # #lines = cv2.HoughLinesP(can, 1, np.pi / 180, threshold, minLineLength, maxLineGap)
        # #if lines is not None:
        #     # for x1, y1, x2, y2 in lines[0]:
        #     #     cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # (cnts, _) = cv2.findContours(g.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
        # screenCnt = None
        #
        # for c in cnts:
        #     # approximate the contour
        #     peri = cv2.arcLength(c, True)
        #     approx = cv2.approxPolyDP(c, 0.01 * peri, True)
        #     cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

        #cv2.imshow('ress', frame)

        #res = np.hstack((frame,line))  # stacking images side-by-side
        #cv2.imshow('res', res)

#roadMask = cv2.bitwise_or(roadMask1, roadMask2)
#         roadMask = cv2.bitwise_or(roadMask, yellowMask)
#         roadMask = cv2.bitwise_or(roadMask, roadMask3)
#         #
#         #
#         roadMask = cv2.erode(roadMask, (5, 5), iterations=7)
#         roadMask = cv2.dilate(roadMask, (5, 5), iterations=7)
#         #
#         road = cv2.bitwise_and(frame, frame, mask=roadMask)
#
#         mask = cv2.inRange(hsvt, lowyel, upyel)
#
#
#
#         gray = cv2.cvtColor(road, cv2.COLOR_BGR2GRAY)
#         (cnts, _) = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#         cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]
#         screenCnt = None
#         for c in cnts:
#             # approximate the contour
#             peri = cv2.arcLength(c, True)
#             approx = cv2.approxPolyDP(c, 0.01 * peri, True)
#             cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)
#
#             # if our approximated contour has four points, then
#             # we can assume that we have found our screen
#             if len(approx) == 4:
#                 screenCnt = approx
#                 break
#
#         # can = cannyBGR(frame)
#
#         #road = frame
#         # can = canny(road)
#         # minLineLength = 0
#         # maxLineGap = 0
#         # lines = cv2.HoughLines(can, 1, np.pi / 180, 100, minLineLength, maxLineGap)
#         # if lines is not None:
#         #     for rho, theta in lines[0]:
#         #         print np.pi/180*10
#         #         a = np.cos(theta)
#         #         b = np.sin(theta)
#         #         x0 = a * rho
#         #         y0 = b * rho
#         #         x1 = int(x0 + 1000 * (-b))
#         #         y1 = int(y0 + 1000 * (a))
#         #         x2 = int(x0 - 1000 * (-b))
#         #         y2 = int(y0 - 1000 * (a))
#         #         if theta<np.pi/180*80 or theta>np.pi/180*100:
#         #             cv2.line(road, (x1, y1), (x2, y2), (0, 0, 255), 2)
#
#         #print lines
#         #cv2.imshow("images", road)
#
#         cv2.imshow("hsv", np.hstack([hsvt, line]))
#
#             # src = cropped
#             #
#             # scale = 1
#             # delta = 0
#             # ddepth = cv2.CV_16S
#             #
#             # src = cv2.GaussianBlur(cropped, (5, 5), 0)
#             # src_gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
#             #
#             # #grad_x = cv2.Scharr(src_gray, ddepth, 1, 0, scale, delta, cv2.BOR DER_DEFAULT);
#             # grad_x = cv2.Sobel(src_gray, ddepth, 1, 0, 3, scale, delta, cv2.BORDER_DEFAULT)
#             # #Scharr(src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT);
#             # grad_y = cv2.Sobel(src_gray, ddepth, 0, 1, 3, scale, delta, cv2.BORDER_DEFAULT)
#             # abs_grad_x = cv2.convertScaleAbs(grad_x)
#             # abs_grad_y = cv2.convertScaleAbs(grad_y)
#             # grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
#             # cv2.imshow("YO", grad)
#
#
#         #cv2.imshow("image", rd)
#        # cv2.imshow("ed", frame)
#         #cv2.imshow("eds", road)
# #<50 l -> 7-20 >50 -> 25-34
#
#
#         # minLineLength = 1
#         # maxLineGap = 100
#         # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength, maxLineGap)
#         # for x1, y1, x2, y2 in lines[0]:
#         #     cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#
#         #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
#
#        # (mu, sigma) = cv2.meanStdDev(stairs8u)
#        # edges = cv2.Canny(stairs8u, mu - sigma, mu + sigma)
#        # lines = cv2.HoughLines(edges, 1, pi / 180, 70)
#
#
#
#
#         #gaussFrame = cv2.GaussianBlur(cannyFrame, (5, 5), 0)
#
#         #roadMask = cv2.inRange(hsv, lower, upper)
#
#         # apply a series of erosions and dilations to the mask
#         # using an elliptical kernel
#         #roadMask = cv2.erode(roadMask, kernel, iterations=2)
#         #roadMask = cv2.dilate(roadMask, kernel, iterations=2)
#
#         # blur the mask to help remove noise, then apply the
#         # mask to the frame
#         #roadMask = cv2.GaussianBlur(roadMask, (3, 3), 0)
#         #road = cv2.bitwise_and(frame, frame, mask=roadMask)
#
#
#         #gray = cv2.cvtColor(roadFrame, cv2.COLOR_BGR2GRAY)
#         #lower_yellow = np.array([20, 100, 100], dtype="uint8")
#         #upper_yellow = np.array([35, 255, 255], dtype="uint8")
#
#
#         # show the skin in the image along with the mask
#         # print hls.shape #200, 249, 3
#         # print hls
#         #
#         # ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
#         # # noise removal
#         # kernel = np.ones((3, 3), np.uint8)
#         # opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
#         # # sure background area
#         # sure_bg = cv2.dilate(opening, kernel, iterations=3)
#         # # Finding sure foreground area
#         # dist_transform = cv2.distanceTransform(opening, cv2.cv.CV_DIST_L2, 5)
#         # ret, sure_fg = cv2.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)
#         # # Finding unknown region
#         # sure_fg = np.uint8(sure_fg)
#         # unknown = cv2.subtract(sure_bg, sure_fg)
#         # # Marker labelling
#         # ret, markers = cv2connectedComponents(sure_fg)
#         # # Add one to all labels so that sure background is not 0, but 1
#         # markers = markers + 1
#         # # Now, mark the region of unknown with zero
#         # markers[unknown == 255] = 0
#         # markers = cv2.watershed(frame, markers)
#         # frame[markers == -1] = [255, 0, 0]
#
#         # roadMask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
#         # roadMask = canny(gray)
#         # roadMask = cv2.erode(roadMask, (2, 2), iterations=2)
#         # roadMask = cv2.dilate(roadMask, (2, 2), iterations=2)
#
#
#
#         #mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
#         #mask_white = cv2.inRange(gray, 200, 255)
#         #mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
#         #mask_yw_image = cv2.bitwise_and(gray, mask_yw)
#         #cv2.imshow('frame',mask_yw_image)
#
#         #cv2.imshow('frame',roadFrame)

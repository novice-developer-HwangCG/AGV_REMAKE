import cv2
import numpy as np

class LineTracer:
    # --- 파라미터(튜닝 가능) ---
    ROI_Y       = 320
    ROI_H       = 160
    THRESH      = 70
    MIN_AREA    = 800
    MAX_AREA    = 50000
    ASPECT_MIN  = 0.2
    ASPECT_MAX  = 5.0
    CLAHE_CLIP  = 2.0
    CLAHE_GRID  = (8, 8)
    MORPH_KERNEL= (3, 3)
    MORPH_ITERS = 1
    DIST_THRESH = 100.0  # 픽셀 단위 최대 이동 허용치

    def __init__(self):
        self.initial_center_set = False
        self.pre_cx = None
        self.pre_cy = None

        # CLAHE 객체
        self.clahe = cv2.createCLAHE(clipLimit=self.CLAHE_CLIP, tileGridSize=self.CLAHE_GRID)

        # Morphology 커널
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, self.MORPH_KERNEL)

    def preprocess(self, roi):
        # 그레이 변환 + CLAHE
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = self.clahe.apply(gray)

        # 이진화 (검은 테이프: THRESH_BINARY_INV)
        _, binary = cv2.threshold(gray, self.THRESH, 255, cv2.THRESH_BINARY_INV)

        # 노이즈 제거 (open → close)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  self.kernel, iterations=self.MORPH_ITERS)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel, iterations=self.MORPH_ITERS)
        return binary

    def filter_contours(self, contours):
        filtered = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if not (self.MIN_AREA < area < self.MAX_AREA):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = h / float(w) if w>0 else 0
            if not (self.ASPECT_MIN < aspect < self.ASPECT_MAX):
                continue
            filtered.append(cnt)
        return filtered

    def select_contour(self, contours):
        # 첫 중심점 설정
        if not self.initial_center_set:
            for cnt in contours:
                if cv2.contourArea(cnt) > self.MIN_AREA:
                    x, y, w, h = cv2.boundingRect(cnt)
                    self.pre_cx = x + w//2
                    self.pre_cy = y + h//2
                    self.initial_center_set = True
                    return cnt
            return None

        # 이후엔 가장 가까운 중심 선택
        min_dist = float('inf')
        best = None
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w//2, y + h//2
            d = np.hypot(cx-self.pre_cx, cy-self.pre_cy)
            if d < min_dist:
                min_dist, best = d, cnt

        # 급격 이동 방지
        if min_dist > self.DIST_THRESH:
            return None
        return best

    def compute_error(self, cx, roi_width):
        center_x = roi_width // 2
        return cx - center_x

    def visualize(self, output, contour):
        x, y, w, h = cv2.boundingRect(contour)
        cx, cy = x + w//2, y + h//2
        cv2.rectangle(output, (x,y), (x+w,y+h), (0,255,0), 2)
        cv2.circle(output, (cx,cy), 5, (0,0,255), -1)

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        # 1) ROI 추출
        roi = frame[self.ROI_Y:self.ROI_Y+self.ROI_H, 0:w]

        # 2) 전처리
        binary = self.preprocess(roi)

        # 3) 컨투어 검출 → 필터링
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = self.filter_contours(contours)

        # 4) 대상 컨투어 선택
        best = self.select_contour(contours)
        error = 0
        vis = frame.copy()
        roi_vis = vis[self.ROI_Y:self.ROI_Y+self.ROI_H, 0:w]

        if best is not None:
            x, y, w_b, h_b = cv2.boundingRect(best)
            cx = x + w_b//2
            cy = y + h_b//2
            self.pre_cx, self.pre_cy = cx, cy

            # 5) 시각화
            self.visualize(roi_vis, best)

            # 6) 에러 계산
            error = self.compute_error(cx, roi.shape[1])

        return vis, error

# --- 사용 예시 ---
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    tracer = LineTracer()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        vis, error = tracer.process_frame(frame)
        print(f"Steering error: {error:+.1f}")

        cv2.imshow("LineTracer", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

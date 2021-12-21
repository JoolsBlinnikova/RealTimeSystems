from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
			
			# img in RGB
            # add here some image processing
            img = cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_BGR2RGB)
            marker = cv2.inRange(img, (0, 130, 170), (2, 250, 255))
            kontur, _ = cv2.findContours(marker.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if kontur:
                _, _, _, h = cv2.boundingRect(kontur[0])
                if h > 130:
                    condition = False
                    img, _, _, _ = env.step([0,0])
            env.render()
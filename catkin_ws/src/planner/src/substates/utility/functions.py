import math

def degreesToVector(angleDegrees):
        angleRadians = (angleDegrees - 90) * math.pi / 18
        x = math.cos(angleRadians)
        y = math.sin(angleRadians)
        return [x, y]

def normalize_vector(vector2D):
        magnitude = math.sqrt(vector2D[0] ** 2 + vector2D[1] ** 2)
        if magnitude == 0:
            return vector2D
        normalized_x = vector2D[0] / magnitude
        normalized_y = vector2D[1] / magnitude
        return [normalized_x, normalized_y]

def dotProduct(v1, v2):
        return v1[0]*v2[0] + v1[1]*v2[1]
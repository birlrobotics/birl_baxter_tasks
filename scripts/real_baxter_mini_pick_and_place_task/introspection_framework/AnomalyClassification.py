from anomaly_classification_proxy.srv import (
    AnomalyClassificationService,
)
import rospy

class AnomalyClassification(object):
    def __init__(self):
        pass

    def classify_anomaly_at(self, anomaly_t):
        sp = rospy.ServiceProxy('AnomalyClassificationService', AnomalyClassificationService)
        resp = sp(anomaly_t)

        return False

import numpy as np 
from numpy.linalg import inv

class Kalman:
  def __init__(self, prior_mean, prior_variance, measurement_mean, measurement_variance):
    #initial robot confidence
    self.prior_mean = prior_mean #u
    self.prior_variance = prior_variance #sigma
    self.measurement_mean = measurement_mean #v
    self.measurement_variance = measurement_variance #r

  #Measurement updates, with measurements probability 
  def updated_mean(self, prior_mean, prior_variance, measurement_mean, measurement_variance):
      updated_mean = ((prior_variance * measurement_mean) + (prior_mean * measurement_variance))/ ((prior_variance) + (measurement_variance))
      return updated_mean #u = Au(t-1) + Bu

  def updated_variance(self, prior_variance, measurement_variance):
      updated_variance = 1 / ((1/prior_variance**2) + (1/measurement_variance**2))
      return updated_variance #Sigma = AEA^t + R_t

  #Prediction: Motion Update with motion distribution and probability
  def predict(self, prior_mean, prior_variance, measurement_mean, measurement_variance):
      updated_mean = prior_mean + measurement_mean
      updated_variance = prior_variance + prior_mean
      return [updated_mean, updated_variance]

  def update(self, measurement_mean, measurement_variance):
    self.prior_mean = self.updated_mean(self.prior_mean, self.prior_variance, .8, .1)
    self.prior_variance = self.updated_variance(self.prior_variance, .1)
    updated_mean, updated_variance = self.predict(self.prior_mean, self.prior_variance, self.measurement_mean, self.measurement_variance)
    print(updated_mean, updated_variance)
    self.prior_variance = variance
    self.prior_mean = updated_mean

if __name__ == '__main__':
  answer = Kalman(1, .1, 1.1, .05)
  answer.update(1,.02)
import numpy as np

class ANN_controller:
   
    def __init__(self, input_size, hidden_size, output_size):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size

        self.W1 = None
        self.W2 = None

        self.b1 = None
        self.b2 = None

    def upload_parameters(self, genome):
        # here I have a list of values (a genome and I have to split it to have W1, b1, W2, b2)
        W1index = self.hidden_size * self.input_size #+ self.hidden_size # added for the bias
        W1 = genome[:W1index] # -self.hidden_size
        self.W1 = np.array(W1).reshape(self.hidden_size, self.input_size)
        #self.b1 = genome[:self.hidden_size]
        W2 = genome[W1index:] # qui aggiungo self.hidden_size W1index + self.hidden_size
        self.W2 = np.array(W2).reshape(self.output_size, self.hidden_size)
        #self.b2 = genome[:self.output_size]


    def forward(self, X):
        
        #W1, b1, W2, b2 = parameters
        #W1, W2 = parameters


        # activation function hidden layer
        Z1 = np.dot(self.W1, np.transpose(X)) #+ b1  # X row vector, Z column vector
        A1 = np.tanh(Z1)
        
        # activation function output layer
        Z2 = np.dot(self.W2, A1) #+ b2
        A2 = np.tanh(Z2)

        return A2

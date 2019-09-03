### (Work in progress...) 

I have prepared this repository to document Kalman filter process. I used a couple of sources to prepare this document as follows:

- [Lectures in the Kalman Filter](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190)
- []()

I am also working on a sample project (in progress) to impletemt this process. 

# 1. Kalman Filter Basics 

Kalman filter is an iterative mathematical process that uses a set of equations and consecutive data inputs to quickly estimate the true value, position, velocity, etc. of the object being measured, when the measured values contain unpredicted or random error, uncertainty, or variation. 

In this process, we start with an initial estimate of a measurement, with certain amount of error or uncertainty. As we receive more data inputs and we go through this iterative process, the Kalman Filter’s estimated value narrows down close to the actual value of the measurement. 

There are three main calculations that need to be done – in an iterative way: 
1. Calculate the Kalman Gain 
2. Calculate the current estimate 
3. Calculate the new error (uncertainty) in the new estimate 

<!--
![Kalman Filter Overview Chart](images/chart-01.jpg")
-->

<a href="images/KF-01.jpg" target="_blank"><img src="images/KF-01.jpg" 
alt="Kalman Filter Overview" width="90%" height="90%" border="10" /></a>

## 1-1. Calculate the Kalman Gain - KG 

There are two main sources of error – uncertainty – in the system: 
  - a. Error in estimate. This error refers to previous error; however, for the initial step, we use original error instead. 
  - b. Error in data input, i.e. observation in the data. 

Kalman gain - KG - applies a relative importance factor – gain – onto these two errors. In other words, it quantifies how much we can trust these two errors, in the estimate and in the data. The result of Kalman Gain is then fed into the estimation of the current estimate. KG is calculated using the following formula:   

\begin{equation*}
KG = \frac{E_{EST}}{E_{EST} + E_{MEA}}
\end{equation*}

where $E_{EST}$ is error in the estimate and $E_{MEA}$ refers to error in the measurement. 

## 1-2. Calculate the Current Estimate 

The current estimate is calculated by three inputs: 

- Previous estimate (or original error at initial step) 
- Error in data input 
- Kalman gain, KG, which is calculated in the first step. 

The equation of updating current estimate is as follows: 

\begin{equation*}
EST_t = EST_{t-1} + KG \cdot (MEA - EST_{t-1})
\end{equation*}

Let us take a closer look at the two extreme values of KG.  

- When $E_{EST}$ is high, $KG$ approaches 1 and therefore $KG$ puts more importance to the measurement, $MEA$, to update the current estimation, $EST_{t-1}$. 
- When $E_{EST}$ is much smaller than $E_{MEA}$, $KG$ approaches 0 and therefore $KG$ puts more importance to the previous estimate, $EST_{t-1}$, to update the current estimation, $EST_{t-1}$. 

Over time, the value of $KG$ should become smaller and smaller and ideally get close to zero, i.e. the estimate become closer to the true value. Therefore, we can rely more and more on estimates rather than measurements, which could be very erratic due to big uncertainty in them. 

## 1-3. Calculate the New Error (Uncertainty) in the New Estimate 

The current estimated error (uncertainty) is calculated as follows: 

\begin{equation*}
E_{EST_{t}} = \frac{(E_{MEA}) \cdot (E_{EST_{t-1}})} {(E_{MEA}) + (E_{EST_{t-1}})}
\end{equation*}

This equation is also often written as follows: 

\begin{equation*}
E_{EST_{t}} = [1 - KG] (E_{EST_{t-1}})
\end{equation*}

Based on this formula we can see that: 

- If the error in the measurement is very small, $KG \rightarrow 1$, the new error, $E_{EST_{t}}$, goes to zero very quickly.  
- If the error in the measurement is very large, $KG \rightarrow 0$, the new error, $E_{EST_{t}}$, goes to zero very slowly. 


<a href="images/KF-Steps.gif" target="_blank"><img src="images/KF-Steps.gif" 
alt="Kalman Filter Overview" width="90%" height="90%" border="10" /></a>

# 2. Kalman Filter – Multi-Dimensional Matrix Equations 

In the first chapter we have seen how Kalman Filter works in a single variable or single state problem. In this section we will see how it works in matrix format. The matrix format means that the information we gather from the system, i.e. measurements, estimates, and their associated uncertainties, will be placed in a matrix format. Similar to the process discussed about above, the Kalman filter process of multiple states includes the steps as follows.  

## 2-1. Start with the Initial State Matrix  

The initial state ($k=0$) of the Kalman filter process contains two components: the state matrix (e.g. position and velocity) and a process covariance matrix (error/uncertainty in the estimate or process). That means that the Kalman filter process inherently accounts for the process error, to keep track of the errors and estimate them in each iteration. The initial and the current state matrices are defined by the equations below. 

\begin{equation*}
\begin{bmatrix}
X_{0} \\
P_{0} 
\end{bmatrix}
\text{    Initial State ,     }
\begin{bmatrix}
X_{k-1} \\
P_{k-1} 
\end{bmatrix}
\text{    Previous State $(k-1)$}
\end{equation*}

An example of state matrix $X$ is position and velocity components of an object in 2D space: 

\begin{equation*}
X = 
\begin{bmatrix}
x \\
y \\ 
\dot {x} \\
\dot {y} \\ 
\end{bmatrix}
\text{    or (in another format)    }
X = 
\begin{bmatrix}
x \\
\dot {x} \\
y \\ 
\dot {y} \\ 
\end{bmatrix}
\end{equation*}

In this process, we also have to take into account the parameter $\Delta t$, time associated with one cycle. 

<a id="section_2-1"></a>

## 2-2. Predict New State from the Previous State  

The state matrix is updated using the following formula: 

\begin{equation*}
\begin{bmatrix}
X_{k} \\
P_{k} 
\end{bmatrix}=
\begin{bmatrix}
A X_{k-1} + B u_{k} + w_{k} \\
A P_{k-1} A^{T} + Q_{k} 
\end{bmatrix}
\end{equation*}

### 2-2-1. State Matrix Representation  

The first part of the equation above is about updating the states of the system using the following parameters: 

- **Control variables matrix**, $u_{k}$, can be taken into account in case we have more detailed information on the physics of the problem, and any stimulating parameter that can impact the status of the states. The more accurate physical model we have from the system, the more accurate (less erratic or uncertain) prediction we can make about the states of the system.  
- **Predicted state noise matrix**, $w_{k}$, can take care of any noise present in the prediction process as well as the errors caused due to deviation from iniital assumptions of the model. For example, if we assume that the velocity of a car is constant within a short period of time $\Delta t$, the error due to deviation from this assumption can be consideerd as noise.  
- **Adaption matrices**, $A$ and $B$, are introduced just to convert inputs to the new state matrix.  
- **Process noise covariance matrix**, $Q_{k}$, is present in updating the process variation to calculate the current state of **process covariance matrix**. The reason why we need to incorporate $Q$ into the process is dsicussed below in section [2-4](#section_2-4)

**An example**: 

Consider a two-dimensional motion with state matrix $X=[x, \dot {x}, y, \dot {y}]$. In order to calculate the next state, the adaptation matrix $A$ will be:  

\begin{equation*}
A = 
\begin{bmatrix}
1 & \Delta t & 0 & 0 \\
0 & 1 & 0 & 0 \\ 
0 & 0 & 1 & \Delta t \\
0 & 0 & 0 & 1 \\ 
\end{bmatrix}
\end{equation*}

In Kalman filter process, the term $A X_{k-1}$ accounts for changes in both $X$ and $\dot {x}$, purely in mathematical sense. Therefore, the change in velocity is calculated without having any more in-depth information on the system. If we have more knowledge about the underlying physics of the system and we would like to incorporate it into the process, we can do it through the next term, $B u_{k}$. 

The reason of spliting these two terms into $A X$ and $B u$ is that Kalman filter process should work even if we do not have any in-depth knowledge on the underlying physics of the system.  

In case of a two-dimensional motion with acceleration, the term $B u$ should look like this: 

\begin{equation*}
\begin{bmatrix}
\frac{1}{2} \Delta t^2 & 0 \\
0 & \frac{1}{2} \Delta t^2 \\ 
\Delta t & 0\\
0 & \Delta t \\ 
\end{bmatrix}
\begin{bmatrix}
a_x \\
a_y \\ 
\end{bmatrix}
\end{equation*}

### 2-2-2. Process Variation Representation  

The second part of the formula is about updating the **new process variation** or **state covariance matrix**, $P$. We would need the state covariance matrix because the entire state-update process is theoretical and therefore it would have some error / uncertainty and noise. Therefore, we have also add the **process noise covariance matrix** to the calculations. This noise can include any stimulus, affecting dynamic of the system and we have not accounted for in our model of the system. Kalman filter process would work only if we account for a reasonable expectation of the error in the process. We also have to take into account the **measurement covariance matrix**, $R$ (discussed below), which contains the error in the measurement. 

**Some Elaboration on Covariance Matrices**: 

Standard deviation of a set of scattered data points, $\sigma_{x}$, is simply squared root of variance, $\sigma_{x}^2$. Standard deviation means that about $2/3$ of the values lie within $\bar {x} \pm SD$. But if we take the valus of variance, instead of SD, we expect to have almost $100\%$ of the data within that range (i.e. value wise, otherwise $3 SD$ would also cover almost $99\%$). For Kalman filter process we use variance to ensure that we open up the gate (in both estimation anf measurement) large enough that we can expect all possible values fall within this range.  
\begin{equation*}
\sigma_{x}^2 = 
\begin{bmatrix}
\frac{\sum \left( x_{i} - \bar {x} \right)^2}{N} \\
\end{bmatrix}
\text{ = variance along x-direction}
\end{equation*}

\begin{equation*}
\sigma_{y}^2 = 
\begin{bmatrix}
\frac{\sum \left( y_{i} - \bar {y} \right)^2}{N} \\
\end{bmatrix}
\text{ = variance along y-direction}
\end{equation*}

Covariance, $\sigma_x \sigma_y$, takes into account both length and width of the data points. In other words, covariance represents the **relative variance**, for example $\sigma_x \sigma_y$ would be variance of $x$ with respect to $y$, and vice versa, $\sigma_y \sigma_x$ would be variance of $y$ with respect to $x$. For example, in two-dimensional space we have: 

\begin{equation*}
\sigma_x \sigma_y = 
\frac{1}{N} \sum {\left( x_{i} - \bar {x} \right) \left( y_{i} - \bar {y} \right)}
\text{ = covariance}
\end{equation*}

Now let us see what a covariance matrix means in the context of Kalman filter process. Let us suppose that we would like to track an object using its position coordinates, $x$, $y$, and their vriations with respect to each other. If we consolidate these values and put them in matrix format, we have to define the covariance matrix. Therefore, the diagonal elements represent variations in the individual measurements, e.g. $x$ or $y$, and the off-diagonal elements represent the co-variance, i.e. the variance related to other measurements, e.g. $x$ and $y$. Following that 2D example, the covariance matrix can be written as follows: 

\begin{equation*}
\begin{bmatrix}
\sigma_{x}^2 & \sigma_{x} \sigma_{y} \\
\sigma_{y} \sigma_{x} & \sigma_{y}^2\\
\end{bmatrix}
\end{equation*}

In matrix form, in order to calculate the covariance matrix based on the input data, we can first find the deviation matrix as defined below 

\begin{equation*}
a = \frac{1}{N} X 
\begin{bmatrix}
1 \\
\end{bmatrix}
X
\end{equation*}

where $X_{N \times n}$ is data point matrix and $[1]$ refers to an $N \times N$ matrix with all elements to be $1$. Then the covariance is simply $a^T a$. 

Another point to hightlight here is that a positive covariance value means direct relationship between two variations, and vice-versa, negative covariance indicates inverse relationship (zero means no dependency). 

#### 2-2-2-1. Original / Initial Error Estimate
Now that we have discused about the definition of covariance matrix, let us see how it is used to the inital calculate process variaions, $P_{k-1}$ in Kalman filter process. To do so, let us take an 2-dimensional example of linear motion with the states $[x, \dot {x}]$ or simply $[x, v_{x}]$, with known initial errors of $\Delta x$ and $\Delta v_{x}$. Therefore, the initial process variaions matrix will be: 

\begin{equation*}
P_{0} = 
\begin{bmatrix}
{\Delta x}^2 & \Delta x \Delta v_{x} \\
\Delta v_{x} \Delta x & {\Delta v_{x}}^2 \\
\end{bmatrix}
\end{equation*}

However, to simplify it, we do not often know the relationship between the change in $x$ and $v_{x}$ adn therefore we do not need off-diagonal terms and we can set them to zero as follows: 

\begin{equation*}
P_{0} = 
\begin{bmatrix}
{\Delta x}^2 & 0 \\
0 & {\Delta v_{x}}^2 \\
\end{bmatrix}
\end{equation*}



## 2-3. Estimate Actual Measured Value  

Once we have done the theoretical prediction, we are ready to incorporate the actual measured value as follows: 

\begin{equation*}
Y_{k} = C X_{k_{M}} + z_{k}
\end{equation*}

where $Y$ referes to the new actual measured value of the states and has the same format as that of state matrix $X$. Matrix $C$ is an **Adaption matrix** (as discussed above), are introduced just to convert inputs to the new state matrix. The last term $z_{k}$ referes to measurement noise or uncertainty, which comes from mechanism of measurement. An example of this noise is the error due the delay in measurement process, e.g. by the time we measure a distance of a satellite, it may have moved from the original position.  


<a id="section_2-4"></a>
## 2-4. Estimate Kalman Gain  

After updating the state prediction and state measurement - as discussed above - we let KG decide (calculate) what fraction of the theoretical and measured values should be contributed into updating the new state. The Kalman gain is a means to assign a weight factor into 'estimation' and 'measured value' (as discussed above). For example, if the error in the observation is large, we would put more valie into the estimation, i.e. $K \rightarrow 0$. In that case, we tend to ignore the observation and observation error as $P \rightarrow 0$. Observation provides valuable source of information and we should not simply miss it because of over-confidence in our estimation. In order to fix this issue we have to add a regularizer, which is $Q$ in the equation above, and it is called the **process noise covariance matrix**. 

Therefore, we feed these two inputs into KG estimation formula as per the equation below. 

\begin{equation*}
K = \frac{P_{k_{p}} H}{H P_{k_{p}} H^{T} + R}
\end{equation*}

where $R$ is the **sensor noise covariance matrix**. 

As an xample, let us again take a 2-dimensional example of linear motion with the states $[x, \dot {x}]$ or simply $[x, v_{x}]$, with observation errors of $\Delta x$ and $\Delta v_{x}$. Therefore, the **sensor noise covariance matrix** will be: 

\begin{equation*}
R = 
\begin{bmatrix}
{\Delta x}^2 & 0 \\
0 & {\Delta v_{x}}^2 \\
\end{bmatrix}
\end{equation*}

We often set the off-diagonal matrices to zero to simplify it, as we do not know the relationship between $[x, \dot {x}]$ and $[x, v_{x}]$ in iterative Kalman filter process. It should be noted that in the division in Kalman gain equation above refers to **by element** division. 

**Note**:
- If in the Kalman filter process, the diagonal elements of **process noise covariance matrix**, $P_{k_{p}}$, become smaller and smaller than the diagonal elements of **sensor noise covariance matrix**, $R$, it means that the Kalman gain pute more emphasis on the predicted values rather than the measured/observed values.  
- Reversly, if **sensor noise covariance matrix**, $R$, becomes smaller and smaller in the measurement process - more accurate observation, Kalman gain will put more emphasis on the observation values. 



## 2-5. Update the State Matrix and Process Error Estimation  

In the last step before starting the iteration, we update the **state matrix** and the **process error estimation** as per equations below: 

\begin{equation*}
X_{k} = X_{k_{p}} + K [Y - H X_{k_{p}}]
\end{equation*}

 and 

\begin{equation*}
P_{k} = \left( I - K H \right) P_{k_{p}}
\end{equation*}

It shoud be noted that by updating the **process error estimation**, we keep track of the error/uncertainty introduced as a part of this KG methodology process. 

## 2-6. Repeat the Cycle for the Next Step  

The **state matrix** and the **process error estimation**, that are updated in the previous step, are then fed into step [2-2](#section_2-1), "Predict New State from the Previous State", to start the next iteration of this process. 

<a href="images/KF-02.jpg" target="_blank"><img src="images/KF-02.jpg" 
alt="Kalman Filter Overview" width="90%" height="90%" border="10" /></a>

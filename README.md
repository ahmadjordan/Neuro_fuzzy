# Hybrid auto-adaptive controller (HAC)
In this work, a neuro-fuzzy system (NFS)-based autonomous and adaptive controller termed as the hybrid auto-adaptive controller (HAC) is developed. HAC combines a simplified NFS (Simp_NFS) and a simplified neural network (Simp\_NN). Unlike conventional NFS, in this study, hyper-plane-shaped clusters (HPSCs) are utilized in Simp_NFS, which has no learning parameters like mean and variance in the antecedent part. Only the consequent parameter needs to be adapted, where the adaptation laws are derived from the Simp_NN. The number of learning parameters in Simp\_NN reduced to one by replacing the weights between the hidden and output layers with their mean value. The robustifying control term of Simp_NN is confirming the uniform asymptotic convergence of tracking error to zero. Besides, structural autonomy in Simp\_NFS is supporting HAC to cope with uncertainties in the nonlinear dynamical systems. By utilizing both the Simp\_NFS and Simp\_NN, HAC has reduced its learning parameters significantly. Therefore, it can produce a fast response with minimal computational resources. The theoretical proof of HAC's stability has been shown with the difference Lyapunov function method. The performance of the proposed HAC is evaluated for trajectory tracking problem in Single-Input and Single-Output (SISO) and Multiple-Input and Multiple-Output (MIMO) nonlinear discrete simulated plants. Also, disturbance and uncertainties are added to test the robustness of HAC. The obtained results clearly indicate that HAC provides better tracking accuracy (improvement in the range of 24.5% to 60.0%) with a simple architecture for both SISO and MIMO systems.

# HAC_Matlab

Paper title:
A Simple Auto-Adaptive Controller for a Class of Nonlinear Uncertain Discrete-Times Systems

Necessary steps:

1. Clone HAC git to your computer, or just download the files.
2. Open Matlab. The code was developed using Matlab 2018b, so if you use an older version, you might get some incompability errors. You can use Matlab 2018b or newer.
3. Execute the following files:

    a) run_SISO.m for example 1
    
    b) run_SISO_robotic_arm.m for example 2
    
    c) run_MIMO.m for example 3
4. To plot the figures as displayed in the paper, go inside the "plotting" folder and execute the following files:
 
   a) plotting_siso.m (to plot trajectory tracking from example 1 and 2)
   
   b) plotting_siso_U.m (to plot control signals from example 1 and 2)
   
   c) plotting_mimo.m (to plot trajectory tracking from example 3)




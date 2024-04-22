# Project Name: Hovercraft: control laws variant

## Description

An hovercraft, also known as an air-cushion vehicle or ACV, is a versatile craft capable of traveling over land, water, mud, or ice surfaces at both high speeds and while stationary. Operating more like aircraft than traditional marine vessels, hovercraft utilize a cushion of high-pressure air between their hulls and the surface below, typically contained within a flexible skirt. They hover at heights ranging from 200 mm to 600 mm and can operate at speeds exceeding 20 knots, traversing gradients of up to 20 degrees. Hovercraft find specialized applications in disaster relief, coastguard, military, survey, sports, and passenger services worldwide.

The project aims to enable a hovercraft to follow a predetermined trajectory. Traditional vessel models account for mechanical and hydrodynamic forces based on Newton’s third law. However, obtaining accurate and tractable models for hydrodynamic forces poses a challenge. Hence, an augmented kinematic model is considered.

## Variant

The primary focus here is:

1. Control laws comparison, (CC).
   Compare various control laws on a model.

## Bibliography

There are two papers (Fantoni et al., 1999; Sira-Ramírez and Aguilar Ibáñez, 2000) associated with this project.

## Requirements and Advises

Here are some tips for reviewing the papers:

- A simplistic model provided by equation (4) of (Fantoni et al., 1999) (model in u, v, r surge, sway, and yaw speed) is trivially flat with flat output (u, v). While advantageous due to the absence of inertia-related parameters, it loses the physical meaning of forces.
  
- Robustness to unmodeled perturbations is considered in subsection 4.2.1 p. 160 of (Sira-Ramírez and Aguilar Ibáñez, 2000) under the form of a wave field effect.
  
- Various guidance laws given in (M. and Fossen, 2008), such as line of sight (LOS), pure pursuit (PP), and constant bearing (CB), are to be compared with flatness-based control the model in equation (4) of (Fantoni et al., 1999).

## References

- Fantoni, I. et al. (1999). "Stabilization of a nonlinear underactuated hovercraft". In: Conference on Decision and Control. Phoenix, Arizona, USA, pp. 2533–2538 (cit. on p. 11).
  
- M., Breivik and T.I. Fossen (2008). "Guidance Laws for Autonomous Underwater Vehicles". In: Underwater Vehicles. Ed. by A.V. Inzartsev. Vienna, Austria: I-Tech, pp. 51–76 (cit. on p. 11).
  
- Sira-Ramírez, H. and C. Aguilar Ibáñez (2000). "On the Control of the Hovercraft System". In: Dynamics and Control 10, pp. 151–163 (cit. on p. 11).

Le passage de la 2D à la 3D ajoute trois degrés de liberté : le mouvement sur l'axe Z (l'altitude) et deux axes de rotation supplémentaires. Le vecteur d'état (la pose) passe de 3 variables $(x, y, \theta)$ à 6 variables $(x, y, z, \phi, \theta, \psi)$.

## Les 3 Matrices de Rotation Fondamentales (Matrices d'Euler)

En 3D, une rotation globale est la combinaison de trois rotations élémentaires autour des axes du repère local. Chaque axe correspond à un angle d'attitude spécifique.

**1. Roulis (Roll - $\phi$) : Rotation autour de l'axe X**
S'incliner sur le côté gauche ou droit. L'axe X reste fixe.


$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos(\phi) & -\sin(\phi) \\ 0 & \sin(\phi) & \cos(\phi) \end{bmatrix}$$

**2. Tangage (Pitch - $\theta$) : Rotation autour de l'axe Y**
Piquer du nez ou se cabrer. L'axe Y reste fixe.


$$R_y(\theta) = \begin{bmatrix} \cos(\theta) & 0 & \sin(\theta) \\ 0 & 1 & 0 \\ -\sin(\theta) & 0 & \cos(\theta) \end{bmatrix}$$

**3. Lacet (Yaw - $\psi$) : Rotation autour de l'axe Z**
Tourner sur soi-même comme une toupie (c'est la matrice utilisée en 2D). L'axe Z reste fixe.


$$R_z(\psi) = \begin{bmatrix} \cos(\psi) & -\sin(\psi) & 0 \\ \sin(\psi) & \cos(\psi) & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**La matrice de rotation 3D complète ($R$)** s'obtient en multipliant ces trois matrices. L'ordre de multiplication est crucial car les rotations en 3D ne sont pas commutatives. En robotique et aéronautique, la convention standard est souvent Z-Y-X (Lacet, puis Tangage, puis Roulis) :


$$R = R_z(\psi) \cdot R_y(\theta) \cdot R_x(\phi)$$

## Transformation Homogène d'un Point 3D

Pour transformer un point détecté par un capteur du repère local ($P_R$) vers le repère global de la carte ($P_W$), la matrice de transformation homogène passe d'une taille 3x3 à une taille 4x4.

Elle encapsule la matrice de rotation complète $R$ (taille 3x3) et le vecteur de translation de l'origine du robot $[X_{robot}, Y_{robot}, Z_{robot}]^T$.

L'équation globale s'écrit :


$$\begin{bmatrix} x_{monde} \\ y_{monde} \\ z_{monde} \\ 1 \end{bmatrix} = \begin{bmatrix} R_{11} & R_{12} & R_{13} & X_{robot} \\ R_{21} & R_{22} & R_{23} & Y_{robot} \\ R_{31} & R_{32} & R_{33} & Z_{robot} \\ 0 & 0 & 0 & 1 \end{bmatrix} \cdot \begin{bmatrix} x_{local} \\ y_{local} \\ z_{local} \\ 1 \end{bmatrix}$$

Où les coefficients $R_{ij}$ sont les éléments résultants de la matrice de rotation combinée $R$.

Bien que les matrices d'Euler soient très visuelles pour comprendre la géométrie, elles souffrent d'une faille mathématique majeure appelée "blocage de cardan" (Gimbal Lock) : lorsque le tangage atteint exactement 90°, les axes de roulis et de lacet s'alignent, faisant perdre un degré de liberté au système. Pour contourner ce problème, les systèmes GNC modernes évitent les matrices de rotation au profit d'outils mathématiques plus robustes.

## 2. Cinématique Simple : Différentiel et Bicyclette

La cinématique est la discipline qui relie les commandes envoyées aux actionneurs (les moteurs) au mouvement géométrique global du robot (vitesse linéaire $V$ et vitesse angulaire $\omega$).

### A. Le robot à entraînement différentiel
C'est l'architecture la plus répandue en robotique de service (ex: TurtleBot, robots aspirateurs). Le robot possède deux roues motrices indépendantes disposées sur le même axe.

*   **Variables du modèle :**
    *   $v_D$ : Vitesse de la roue droite
    *   $v_G$ : Vitesse de la roue gauche
    *   $L$ : Entraxe (distance séparant les deux roues)

*   **Équations fondamentales :**
    *   La vitesse linéaire globale du robot ($V$) est la moyenne des vitesses des deux roues :
        $$V = \frac{v_D + v_G}{2}$$
    *   La vitesse angulaire du robot ($\omega$) est proportionnelle à la différence de vitesse des roues :
        $$\omega = \frac{v_D - v_G}{L}$$

*   **Comportement :**
    *   Si $v_D = v_G$ : Mouvement rectiligne pur ($V > 0$, $\omega = 0$).
    *   Si $v_D = -v_G$ : Rotation pure sur place ($V = 0$, $\omega \neq 0$).

### B. Le modèle bicyclette (Véhicule type Ackermann)
Contrairement au robot différentiel, une voiture classique est soumise à des contraintes non holonomes (elle ne peut pas se déplacer latéralement ni tourner sur place). On modélise souvent ce système par un "vélo" : une roue arrière propulsive et une roue avant directrice.

*   **Variables du modèle :**
    *   $V$ : Vitesse de propulsion (roue arrière)
    *   $L$ : Empattement (distance entre l'essieu avant et arrière)
    *   $\delta$ : Angle de braquage de la roue avant

*   **Équations fondamentales :**
    *   La vitesse de translation du centre du véhicule correspond directement à $V$.
    *   La vitesse angulaire ($\omega$) de rotation du châssis est contrainte par l'angle de braquage et la vitesse d'avancement :
        $$\omega = \frac{V}{L} \cdot \tan(\delta)$$

*   *Note importante :* Dans ce modèle, si le véhicule est à l'arrêt ($V = 0$), sa vitesse angulaire est obligatoirement nulle ($\omega = 0$), quelle que soit la valeur de braquage $\delta$.

---

## 3. Mise à jour de l'état : L'Odométrie
Une fois les vitesses $V$ et $\omega$ calculées par le modèle cinématique, le robot doit intégrer ces vitesses dans le temps pour mettre à jour sa position sur la carte. C'est le principe de l'odométrie (par la méthode d'Euler).

À chaque pas de temps $\Delta t$ de la boucle de contrôle, les nouvelles coordonnées se calculent ainsi :

$$x_{nouveau} = x_{ancien} + V \cdot \cos(\theta_{ancien}) \cdot \Delta t$$
$$y_{nouveau} = y_{ancien} + V \cdot \sin(\theta_{ancien}) \cdot \Delta t$$
$$\theta_{nouveau} = \theta_{ancien} + \omega \cdot \Delta t$$
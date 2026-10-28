Pour formuler l'ensemble du système de contrôle, définissons d'abord l'opérateur mathématique générique d'un régulateur PID pour une erreur $e(t)$ :

$$PID(e) = K_p e + K_i \int e \, dt + K_d \frac{de}{dt}$$

*(Note : Dans l'industrie, comme on l'a vu, on met souvent $K_i = 0$ ou $K_d = 0$ selon l'étage pour obtenir un simple proportionnel P ou PI).*

Voici la cascade complète des équations, du plus lent (navigation 3D) au plus rapide (moteurs), en séparant les axes dynamiques.

### 1. Contrôle de l'Altitude (Axe Z)

L'altitude est indépendante de l'inclinaison. On génère directement la poussée globale (Thrust).

* **Erreur de position Z :** $e_z = Z_{cible} - Z_{mesure}$
* **Consigne de vitesse verticale :** $V_{z,cible} = P(e_z)$
* **Erreur de vitesse verticale :** $e_{vz} = V_{z,cible} - V_{z,mesure}$
* **Commande de Poussée Totale ($T_c$) :**

$$T_c = PID(e_{vz}) + m \cdot g$$



*(Le terme $m \cdot g$ est le Feedforward de gravité : on donne de base aux moteurs l'ordre de contrer le poids du drone pour qu'il plane à vitesse nulle).*

### 2. Contrôle Latéral et Longitudinal (Axes X et Y)

C'est ici que la position se transforme en inclinaison.

* **Erreurs de position plan :**

$$e_x = X_{cible} - X_{mesure}$$


$$e_y = Y_{cible} - Y_{mesure}$$


* **Consignes de vitesses horizontales :**

$$V_{x,cible} = P(e_x)$$


$$V_{y,cible} = P(e_y)$$


* **Erreurs de vitesses :**

$$e_{vx} = V_{x,cible} - V_{x,mesure}$$


$$e_{vy} = V_{y,cible} - V_{y,mesure}$$


* **Accélérations désirées :**

$$A_{x,cible} = PID(e_{vx})$$


$$A_{y,cible} = PID(e_{vy})$$



**La conversion clé (Accélération vers Angles) :**
Pour accélérer en $X$ (avant), le drone s'incline en tangage ($\theta$). Pour accélérer en $Y$ (gauche/droite), il s'incline en roulis ($\phi$). Sous l'hypothèse des petits angles, les consignes d'attitude deviennent approximativement :


$$\theta_{cible} \approx \frac{A_{x,cible}}{g}$$

$$\phi_{cible} \approx -\frac{A_{y,cible}}{g}$$

### 3. Contrôle de Cap (Axe Lacet / Yaw)

Le lacet gère uniquement l'orientation autour de l'axe Z.

* **Erreur de cap :** $e_{\psi} = \psi_{cible} - \psi_{mesure}$
* **Consigne de rotation lacet :** $r_{cible} = P(e_{\psi})$

### 4. Contrôle d'Attitude (Angles)

On rassemble les angles demandés ($\phi, \theta$) et on les compare à l'inclinaison réelle.

* **Erreurs d'attitude :**

$$e_{\phi} = \phi_{cible} - \phi_{mesure}$$


$$e_{\theta} = \theta_{cible} - \theta_{mesure}$$


* **Consignes de taux angulaires (p, q) :**

$$p_{cible} = P(e_{\phi})$$


$$q_{cible} = P(e_{\theta})$$



### 5. Contrôle des Taux Angulaires (Boucle interne rapide)

On compare les vitesses de rotation demandées ($p_{cible}, q_{cible}, r_{cible}$) avec les mesures brutes des gyroscopes ($p_{mes}, q_{mes}, r_{mes}$).

* **Erreurs gyroscopiques :**

$$e_p = p_{cible} - p_{mesure}$$


$$e_q = q_{cible} - q_{mesure}$$


$$e_r = r_{cible} - r_{mesure}$$


* **Commandes de Couples (Torques) :**

$$\tau_x = PID(e_p)$$


$$\tau_y = PID(e_q)$$


$$\tau_z = PID(e_r)$$



---

### 6. Le Mixer (La matrice de mixage finale)

À ce stade, le contrôleur a produit quatre forces abstraites : la poussée totale $T_c$ et les trois couples $\tau_x, \tau_y, \tau_z$.
Le Mixer convertit ces commandes en signaux individuels pour chaque moteur (ex: $M_1$ à $M_4$ pour un quadricoptère en configuration "X"), généralement via une matrice constante qui dépend de la géométrie du châssis :

$$\begin{bmatrix} M_1 \\ M_2 \\ M_3 \\ M_4 \end{bmatrix} =  \begin{bmatrix}  1 & -1 &  1 &  1 \\  1 &  1 & -1 &  1 \\  1 &  1 &  1 & -1 \\  1 & -1 & -1 & -1  \end{bmatrix} \begin{bmatrix} T_c \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}$$
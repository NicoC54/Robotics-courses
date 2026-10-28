# ARCHITECTURE GNC GLOBALE (GUIDANCE, NAVIGATION, CONTROL) D'UN DRONE AUTONOME 6-DDL

C'est la "Big Picture" ultime d'un drone autonome moderne (type PX4 / ArduPilot couplé à un ordinateur compagnon ROS2). Elle relie l'intelligence artificielle de planification (**A\*, RRT\***), le pilotage automatique (**les boucles PID en cascade et le Mixeur**) et la modélisation physique (**la Dynamique 6-DDL et l'EKF**).

---

## La Philosophie du GNC : Les 3 Piliers et leurs Fréquences

Le cerveau du drone est divisé en 3 départements qui travaillent en boucle fermée, chacun à son propre rythme temporel (du plus lent et réfléchi au plus rapide et réflexe) :

1. **GUIDANCE (Planification & Guidage - *Où veut-on aller et par quel chemin ?* | $1\text{ Hz}$ à $50\text{ Hz}$) :** Analyse l'environnement (cartes, LiDAR, caméras), évite les obstacles via des algorithmes de recherche de chemin ($\text{A}^*$, $\text{RRT}^*$) et génère une trajectoire lisse sous forme de consignes de position, vitesse et accélération dans le repère Monde $W$.
2. **CONTROL (Commande & Mixage - *Quels ordres donner aux moteurs ?* | $50\text{ Hz}$ à $1000\text{ Hz}$) :** Compare la consigne de trajectoire avec l'état estimé par l'EKF, calcule les efforts requis ($\vec{U}$) via des boucles PID en cascade, puis les convertit en vitesses moteurs ($\Omega_i$). **Attention :** Le contrôleur utilise l'*inverse* de la matrice d'allocation ($M^{-1}$) !
3. **NAVIGATION (Estimation EKF - *Où suis-je vraiment ?* | $200\text{ Hz}$ à $1000\text{ Hz}$) :** Fusionne en temps réel la prédiction mathématique (pilotée par l'IMU) et les capteurs absolus bruités (GPS, Mag, Baro) pour fournir un vecteur d'état propre $\hat{X}_{\text{est}}$ au contrôleur et au planificateur.

---

## Le Schéma d'Architecture Complet (De A à Z)

### 1. GUIDANCE : PLANIFICATION DE TRAJECTOIRE (Cerveau Haut Niveau)

> #### 🗺️ [A. PLANIFICATEUR GLOBAL (Global Planner)] — *Fréquence : $\sim 0.1$ à $1\text{ Hz}$ (Carte / Octomap)*
> * **Algorithmes :** $\text{A}^*$ (A-Star), Dijkstra, $\text{RRT}^*$
> * **Action :** Trouve le plus court chemin géométrique entre le départ $A$ et la cible $B$ (Waypoints bruts).

$$\Downarrow \quad \text{Suite de points de passage } [\text{WP}_1, \text{WP}_2, \dots, \text{WP}_n]$$

> #### 🌲 [B. PLANIFICATEUR LOCAL & ÉVITEMENT (Local Planner)] — *Fréquence : $\sim 10$ à $30\text{ Hz}$ (Vision / LiDAR)*
> * **Algorithmes :** $\text{A}^*$ local, DWA (*Dynamic Window*), Champs de Potentiel, EGO-Planner
> * **Action :** Déforme le chemin global en temps réel pour contourner les obstacles imprévus (arbres, murs...).

$$\Downarrow \quad \text{Chemin local sans collision}$$

> #### 📈 [C. GÉNÉRATEUR DE TRAJECTOIRE (Trajectory Generator)] — *Fréquence : $\sim 50\text{ Hz}$ (Polynômes Minimum Snap)*
> * **Problème :** Un drone ne peut pas suivre des lignes brisées à cause de son inertie.
> * **Action :** Lisse les points en une courbe temporelle continue respectant la physique ($v_{\max}, a_{\max}$).

$$\Downarrow \quad \text{Consignes glissantes dans } W :$$

$$
\begin{cases} 
\text{Position : } \vec{p}_{\text{ref}}^W(t) = [x_d, y_d, z_d]^T \\ 
\text{Vitesse : } \vec{v}_{\text{ref}}^W(t) = [v_{xd}, v_{yd}, v_{zd}]^T \\ 
\text{Accél. Feed-Forward : } \vec{a}_{\text{ff}}^W(t) \quad \text{et Cap : } \psi_{\text{ref}}(t) 
\end{cases}
$$

---

### 2. CONTROL : BOUCLES PID EN CASCADE & MIXAGE

> #### 🎯 [BOUCLE EXTERNE : Contrôle de Position & Vitesse] — *Fréquence : $\sim 50\text{ Hz}$ (Repère Monde $W$)*
> * **Erreur :** $\vec{e}_p = \vec{p}_{\text{ref}}^W - \vec{p}_{\text{est}}^W$ et $\vec{e}_v = \vec{v}_{\text{ref}}^W - \vec{v}_{\text{est}}^W \longrightarrow \text{[ Correcteur PID + Feed-Forward ]}$
> * **Sortie :** Accélération désirée dans le Monde $\vec{a}_{\text{des}}^W = [a_{xd}, a_{yd}, a_{zd}]^T$
> * **Inversion Géométrique (Gravité $\vec{g}^W$ + Cap $\psi_{\text{ref}}$) :** *Pour accélérer horizontalement de $(a_{xd}, a_{yd})$ tout en compensant $\vec{g}^W$, de combien dois-je m'incliner et pousser ?*

$$\Downarrow \quad \text{Séparation Poussée / Orientation}$$

**Branche 1 : Poussée Totale (Directe)**
> * **Poussée totale requise :**
> $$U_1 = T_{\text{tot}} = m \|\vec{a}_{\text{des}}^W - \vec{g}^W\|$$
> *(Cette commande $U_1$ descend directement vers le Mixeur)*

**Branche 2 : Cascade Angulaire (Attitude $\rightarrow$ Vitesses)**
> * **Angles désirés :** $\vec{\eta}_{\text{ref}} = [\phi_d, \theta_d, \psi_{\text{ref}}]^T$
>
> $$\Downarrow$$
>
> * **[BOUCLE INTERMÉDIAIRE : Contrôle d'Attitude]** *(~250 Hz)*
>   * **Erreur :** $\vec{e}_\eta = \vec{\eta}_{\text{ref}} - \vec{\eta}_{\text{est}} \longrightarrow \text{[ Correcteur P ]}$
>   * **Sortie :** Vitesses angulaires désirées $\vec{\omega}_{\text{ref}}^B = [p_d, q_d, r_d]^T$
>
> $$\Downarrow$$
>
> * **[BOUCLE INTERNE : Vitesses Angulaires]** *(~1000 Hz - Repère $B$)*
>   * **Erreur :** $\vec{e}_\omega = \vec{\omega}_{\text{ref}}^B - \vec{\omega}_{\text{imu}}^B \longrightarrow \text{[ Correcteur PID Vitesse ]}$
>   * **Sortie :** Moments requis $[U_2, U_3, U_4]^T = [\tau_\phi, \tau_\theta, \tau_\psi]^T$

$$\Downarrow \quad \text{Regroupement du Torseur de Commande}$$

$$
\text{Vecteur de Commande Virtuel : } \vec{U} = \begin{bmatrix} T_{\text{tot}} \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix}
$$

> #### ⚙️ [ALLOCATION DE COMMANDE INVERSE (Le "Mixeur" $M^{-1}$)]
> * **Problème :** Le PID veut une poussée et 3 moments ($\vec{U}$). Quelle vitesse donner à chaque moteur ?
> * **Formule :** On **INVERSE** la matrice d'allocation :

$$
\begin{bmatrix} \Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2 \end{bmatrix} = M^{-1} \cdot \begin{bmatrix} T_{\text{tot}} \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix}
$$

$$\Downarrow \quad \text{Commandes PWM / DShot : Vitesses moteurs } [\Omega_1, \Omega_2, \Omega_3, \Omega_4]$$

---

### 3. MONDE PHYSIQUE RÉEL (Le Drone, l'Air et la Gravité)

> #### 🌍 [DYNAMIQUE 6-DDL DU CORPS RIGIDE]
> 1. **Aérodynamique des hélices (Matrice Directe $M$) :** 
> 
> $$
> \vec{U}_{\text{réel}} = M \cdot \begin{bmatrix} \Omega_1^2 & \Omega_2^2 & \Omega_3^2 & \Omega_4^2 \end{bmatrix}^T = \sum_{i=1}^{N} \begin{bmatrix} k_f \\ -y_i k_f \\ +x_i k_f \\ \sigma_i k_m \end{bmatrix} \Omega_i^2
> $$
> 
> 2. **Rotation (TMD dans $B$ + Théorème de Bour) :** 
> 
> $$
> I_B \dot{\vec{\omega}}_B = \vec{\tau}_B - \vec{\omega}_B \times (I_B \vec{\omega}_B) - \vec{\tau}_{\text{gyro}}^B - K_r \vec{\omega}_B
> $$
> 
> 3. **Translation (PFD dans $W$) :** 
> 
> $$
> m \ddot{\vec{p}}_W = R_B^W(\phi, \theta, \psi) \begin{bmatrix} 0 \\ 0 \\ -T_{\text{tot}} \end{bmatrix} + \begin{bmatrix} 0 \\ 0 \\ mg \end{bmatrix} - K_d \dot{\vec{p}}_W
> $$

$$\Downarrow \quad \text{Acquisition par les Capteurs Embarqués}$$

**Capteurs Rapides ($1000\text{ Hz}$ - Repère $B$)**
> * **Gyroscope :** $\vec{\omega}_{\text{imu}}^B = [p, q, r]^T + \vec{b}_g + \text{bruit}$
> * **Accéléromètre :** $\vec{a}_{\text{imu}}^B = R_W^B(\ddot{\vec{p}}_W - \vec{g}^W) + \vec{b}_a + \text{bruit}$

**Capteurs Lents ($10$ à $50\text{ Hz}$ - Repères $W/B$)**
> * **GPS / Baromètre :** $\vec{z}_{\text{gps}} = [\vec{p}^W, \vec{v}^W]^T$
> * **Magnétomètre :** $\vec{z}_{\text{mag}} = R_W^B(\phi,\theta,\psi) \cdot \vec{m}^W$

---

### 4. NAVIGATION : FILTRE DE KALMAN ÉTENDU (EKF)

> #### 🧭 [ÉTAPE A : PRÉDICTION (À $200\text{–}1000\text{ Hz}$, pilotée par l'IMU)]
> 1. **Initialisation :** Utilise les anciens angles estimés $(\phi, \theta, \psi)$ pour remplir $W_\eta^{-1}(\phi,\theta)$ et $R_B^W(\phi,\theta,\psi)$.
> 2. **Intègre le Gyroscope :** 
> 
> $$
> \dot{\vec{\eta}} = W_\eta^{-1}(\phi, \theta) \cdot (\vec{\omega}_{\text{imu}}^B - \vec{b}_g)
> $$
> 
> 3. **Projette l'Accéléromètre dans World :** 
> 
> $$
> \dot{\vec{v}}^W = R_B^W(\phi, \theta, \psi) \cdot (\vec{a}_{\text{imu}}^B - \vec{b}_a) + \vec{g}^W
> $$
> 
> 4. **Intègre pour prédire l'état $\hat{X}_{k|k-1}$ et linéarise via la Jacobienne $F_k$ :**
> 
> $$
> P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k
> $$

$$\Downarrow \quad \text{État prédit } \hat{X}_{k|k-1} \text{ (Très réactif, mais dérive lentement !)}$$

> #### 🎯 [ÉTAPE B : CORRECTION / MISE À JOUR (Dès qu'une mesure GPS / Mag / Baro arrive)]
> 1. **Innovation (Erreur de mesure) :** 
> 
> $$
> \vec{y}_k = \vec{z}_k - h(\hat{X}_{k|k-1})
> $$
> 
> 2. **Gain de Kalman $K_k$ :** 
> 
> $$
> K_k = P_{k|k-1} H_k^T \left( H_k P_{k|k-1} H_k^T + R_k \right)^{-1}
> $$
> 
> 3. **Correction de l'état :** 
> 
> $$
> \hat{X}_{\text{est}} = \hat{X}_{k|k-1} + K_k \vec{y}_k
> $$

$$\Downarrow$$

**État Estimé Propre :**
$$
\hat{X}_{\text{est}} = \left[ \vec{p}_{\text{est}}^W, \, \vec{v}_{\text{est}}^W, \, \vec{\eta}_{\text{est}}, \, \vec{b}_g, \, \vec{b}_a \right]^T \quad (\text{et } \vec{\omega}_{\text{imu}}^B \text{ filtré})
$$

$$\circlearrowleft \quad \mathbf{\text{[ RETOUR EN HAUT VERS GUIDANCE (1) ET LES BOUCLES PID (2) ! ]}}$$

---

## Les 4 Révélations de ce Schéma Complet

### 1. Pourquoi l'A\* et la planification sont séparés des PID ? (La hiérarchie temporelle)
Un algorithme comme $\text{A}^*$ cherche un chemin dans une grille 3D en testant des milliers de nœuds : cela prend du temps de calcul ($10$ à $100\text{ ms}$) et produit une ligne brisée (des angles vifs d'une case à l'autre).
* Si tu envoyais directement des angles vifs au PID, le drone donnerait des à-coups violents. C'est pour cela qu'on intercale un **Générateur de Trajectoire** (souvent polynomial, dit *Minimum Snap*) qui transforme les points de l'$\text{A}^*$ en une courbe fluide $\left(\vec{p}_{\text{ref}}(t), \vec{v}_{\text{ref}}(t), \vec{a}_{\text{ff}}(t)\right)$ évaluée à $50\text{ Hz}$.
* Pendant que l'$\text{A}^*$ réfléchit au prochain virage à $10\text{ Hz}$, la boucle interne PID et l'EKF tournent à **$1000\text{ Hz}$** pour empêcher le drone de se retourner à cause d'une rafale de vent.

### 2. Le double rôle de ta Matrice d'Allocation $M$ (Le Miroir)
Regarde bien où intervient la matrice d'allocation dans le schéma :
* **Dans le Contrôleur (Le Mixeur) :** Les PID calculent la poussée $T_{\text{tot}}$ et les moments $(\tau_\phi, \tau_\theta, \tau_\psi)$ dont le drone a *besoin*. Pour savoir à quelle vitesse faire tourner les moteurs, l'ordinateur utilise **l'inverse de ta matrice ($M^{-1}$)**.
* **Dans la Physique (La Réalité) :** Les moteurs tournent à la vitesse $\Omega_i$, et l'air applique **ta matrice directe ($M$)** pour transformer cette vitesse en poussée et moments réels sur le châssis.

### 3. Pourquoi les PID sont "en cascade" (Poupées russes) ?
Le contrôleur ne passe pas directement de l'erreur de position GPS aux moteurs. Il procède exactement **à l'envers de la physique** :
1. **Pour changer de Position ($x, y$)** $\rightarrow$ Il faut créer une **Accélération horizontale** ($\vec{a}_{\text{des}}^W$).
2. **Pour créer une Accélération horizontale** $\rightarrow$ Il faut pencher le vecteur poussée des hélices, donc demander un **Angle d'Euler ($\phi_d, \theta_d$)** ! *(C'est le PID Externe à $50\text{ Hz}$)*.
3. **Pour atteindre cet Angle d'Euler** $\rightarrow$ Il faut demander une **Vitesse de rotation ($p_d, q_d, r_d$)** ! *(C'est le correcteur P Intermédiaire à $250\text{ Hz}$)*.
4. **Pour atteindre cette Vitesse de rotation** $\rightarrow$ Il faut créer une accélération angulaire, donc demander des **Moments ($\tau_\phi, \tau_\theta, \tau_\psi$)** ! *(C'est le PID Interne à $1000\text{ Hz}$)*.

C'est la symétrie exacte de ton modèle dynamique (Moments $\rightarrow$ Accél. angulaire $\rightarrow$ Vitesse angulaire $\rightarrow$ Angles $\rightarrow$ Poussée inclinée $\rightarrow$ Accél. linéaire $\rightarrow$ Position), mais parcourue en sens inverse !

### 4. Comment tout se reboucle en cercle grâce à l'EKF
Bien qu'un schéma sur une feuille se lise de haut en bas, le système fonctionne comme un **cercle infini** :
* L'EKF (en bas) récupère le résultat chaotique du monde réel via les capteurs.
* Il utilise tes matrices $W_\eta^{-1}$ et $R_B^W$ à $1000\text{ Hz}$ dans l'**Étape A (Prédiction)** pour suivre les mouvements ultra-rapides du drone.
* Il utilise les **Jacobiennes $F_k$ et $H_k$** dans l'**Étape B (Correction)** pour recaler les angles, la position et les biais capteurs dès que le GPS parle.
* Enfin, il renvoie cet état propre $\hat{X}_{\text{est}}$ tout en haut du schéma (pour que l'$\text{A}^*$ sache où commence son chemin, et que les PID calculent leurs erreurs), **ET** il s'en sert lui-même pour mettre à jour ses propres matrices $R_B^W$ et $W_\eta^{-1}$ à la milliseconde suivante !
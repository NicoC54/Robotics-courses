# Fiche de Synthèse : Du PFD/TMD aux Commandes Moteurs d'un Drone (Conventions Aéronautiques NED / FRD & Feedforward)

## Introduction : Le Problème du "Sous-Actionnement"

Un drone quadricoptère possède **6 degrés de liberté** dans l'espace :
* **3 translations :** Position $(x, y, z)$ dans le repère terrestre.
* **3 rotations :** Roulis / *Roll* ($\phi$), Tangage / *Pitch* ($\theta$), Lacet / *Yaw* ($\psi$).

Pourtant, il ne possède que **4 moteurs**. Il est donc **sous-actionné**.
**Conséquence fondamentale :** Un drone ne peut pas se déplacer horizontalement (selon $x$ ou $y$) sans s'incliner. Ses 4 hélices sont fixes par rapport au châssis et ne poussent que dans **une seule direction par rapport au drone** : perpendiculairement au châssis, vers le "haut" du drone (soit $-\vec{z}_B$ en convention FRD).

Pour suivre une trajectoire avec précision sans retard de phase, le contrôleur utilise une **architecture en cascade** couplant des correcteurs **PID (Feedback)** et des termes d'**Anticipation (Feedforward - FF)** :

$$
\text{Trajectoire } (\vec{p}_{\text{ref}}, \vec{v}_{\text{ref}}, \vec{a}_{\text{ref}}) \xrightarrow{\text{PID + FF}} \vec{a}_{\text{des}} \xrightarrow{\text{Inversion PFD}} (T_d, \phi_d, \theta_d) \xrightarrow{\text{PID + FF Attitude}} \vec{\tau}_m \xrightarrow{\text{Mixage}} (f_1, f_2, f_3, f_4)
$$

---

## 1. Définition des Repères Aéronautiques : NED et FRD

En aéronautique (standards PX4, ArduPilot), l'axe $Z$ est toujours dirigé **vers le bas**. Cela change radicalement les signes de la gravité, de la poussée et des angles par rapport aux conventions robotiques classiques.

### A. Le Repère Inertiel Terrestre $\mathcal{R}_I (O, \vec{x}_I, \vec{y}_I, \vec{z}_I)$ — Convention **NED** (*North-East-Down*)
* $\vec{x}_I$ : Pointe vers le **Nord** (Devant).
* $\vec{y}_I$ : Pointe vers l'**Est** (Droite).
* $\vec{z}_I$ : Pointe vers le **Bas** (*Down*, vers le centre de la Terre).
* **Conséquence sur l'altitude et la gravité :**
  * Si le drone vole à $10\text{ m}$ de hauteur, sa coordonnée $z$ vaut $z = -10\text{ m}$. Monter correspond à une vitesse $\dot{z} < 0$ et une accélération $\ddot{z} < 0$.
  * Le vecteur accélération de la pesanteur est **positif** selon $\vec{z}_I$ :
    $$
    \vec{g}_I = \begin{pmatrix} 0 \\ 0 \\ +g \end{pmatrix}_{\mathcal{R}_I} \quad (\text{avec } g \approx 9,81\text{ m/s}^2)
    $$

### B. Le Repère Lié au Drone (*Body*) $\mathcal{R}_B (G, \vec{x}_B, \vec{y}_B, \vec{z}_B)$ — Convention **FRD** (*Forward-Right-Down*)
Attaché au centre de gravité $G$ du drone, il tourne avec lui :
* $\vec{x}_B$ : Pointe vers le **Nez** du drone (*Forward*).
* $\vec{y}_B$ : Pointe vers le bras **Droit** (*Right*).
* $\vec{z}_B$ : Pointe vers le **Ventre** du drone (*Down*).
* **Conséquence sur la poussée totale :** Comme les hélices soufflent l'air vers le bas ($+\vec{z}_B$) pour propulser le drone vers le haut ($-\vec{z}_B$), le vecteur force de poussée totale $\vec{F}_{\text{poussée}}$ (de norme $T \ge 0$) est **négatif** sur l'axe $\vec{z}_B$ :
  $$
  \vec{F}_{\text{poussée}, B} = \begin{pmatrix} 0 \\ 0 \\ -T \end{pmatrix}_{\mathcal{R}_B}
  $$

### C. Sens Physique des Angles d'Euler $(\phi, \theta, \psi)$ en FRD (Règle de la main droite)
1. **Roulis / Roll ($\phi$ autour de $\vec{x}_B$) :**
   * $\phi > 0$ : L'aile droite s'abaisse ($+\vec{z}_I$), l'aile gauche se lève. Le vecteur poussée s'incline vers la droite, le drone accélère vers la **droite** ($+\vec{y}_B$).
2. **Tangage / Pitch ($\theta$ autour de $\vec{y}_B$) — *Attention au signe !* :**
   * $\theta > 0$ : Le drone **cabre** (le nez se lève vers $-\vec{z}_I$, l'arrière s'abaisse). Le vecteur poussée pointe vers l'arrière : le drone **recule** ($-\vec{x}_B$).
   * $\theta < 0$ : Le drone **pique du nez**. Le vecteur poussée pointe vers l'avant : le drone **avance** ($+\vec{x}_B$).
3. **Lacet / Yaw ($\psi$ autour de $\vec{z}_B$) :**
   * $\psi > 0$ : Vu du dessus, le nez du drone tourne vers la **droite** (sens horaire, du Nord vers l'Est).

### D. La Matrice de Rotation $R_{B \to I}$ (Convention Z-Y-X)
Pour passer un vecteur du repère *Body* (FRD) vers le repère *Terrestre* (NED), on applique successivement le roulis $R_x(\phi)$, le tangage $R_y(\theta)$, puis le lacet $R_z(\psi)$ :

$$
R_{B \to I} = R_z(\psi) R_y(\theta) R_x(\phi) = \begin{pmatrix} \cos\psi\cos\theta & \cos\psi\sin\theta\sin\phi - \sin\psi\cos\phi & \cos\psi\sin\theta\cos\phi + \sin\psi\sin\phi \\ \sin\psi\cos\theta & \sin\psi\sin\theta\sin\phi + \cos\psi\cos\phi & \sin\psi\sin\theta\cos\phi - \cos\psi\sin\phi \\ -\sin\theta & \cos\theta\sin\phi & \cos\theta\cos\phi \end{pmatrix}
$$

---

## 2. Boucle Externe de Position : PID + Feedforward (en NED)

On souhaite que le drone suive une trajectoire de référence lisse définie dans le repère terrestre NED par :
* La position de référence : $\vec{p}_{\text{ref}}(t) = (x_{\text{ref}}, y_{\text{ref}}, z_{\text{ref}})^T$
* La vitesse de référence : $\vec{v}_{\text{ref}}(t) = (\dot{x}_{\text{ref}}, \dot{y}_{\text{ref}}, \dot{z}_{\text{ref}})^T$
* L'accélération de référence (**Feedforward**) : $\vec{a}_{\text{ref}}(t) = (\ddot{x}_{\text{ref}}, \ddot{y}_{\text{ref}}, \ddot{z}_{\text{ref}})^T$

En mesurant la position réelle $\vec{p} = (x, y, z)^T$ et la vitesse réelle $\vec{v} = (\dot{x}, \dot{y}, \dot{z})^T$, on forme les erreurs :
$$
\vec{e}_p = \vec{p}_{\text{ref}} - \vec{p} \quad \text{et} \quad \vec{e}_v = \vec{v}_{\text{ref}} - \vec{v}
$$

Le contrôleur de position calcule **l'accélération désirée** $\vec{a}_{\text{des}}$ dans le repère NED en additionnant la correction **PID (Feedback)** et l'accélération de la trajectoire (**Feedforward**) :

$$
\begin{pmatrix} a_{x,\text{des}} \\ a_{y,\text{des}} \\ a_{z,\text{des}} \end{pmatrix}_{\text{NED}} = \underbrace{\boldsymbol{K_p} (\vec{p}_{\text{ref}} - \vec{p}) + \boldsymbol{K_d} (\vec{v}_{\text{ref}} - \vec{v}) + \boldsymbol{K_i} \int (\vec{p}_{\text{ref}} - \vec{p}) \, dt}_{\text{Terme Feedback (PID : corrige les perturbations)}} + \underbrace{\begin{pmatrix} a_{x,\text{ref}} \\ a_{y,\text{ref}} \\ a_{z,\text{ref}} \end{pmatrix}_{\text{NED}}}_{\substack{\text{Terme Feedforward} \\ \text{(Anticipation dynamique)}}}
$$

> **Rôle physique du Feedforward $\vec{a}_{\text{ref}}$ :** Sans ce terme, le drone devrait d'abord prendre du retard sur sa trajectoire ($\vec{e}_p \neq \vec{0}$) pour que le PID commence à commander une accélération. Avec $\vec{a}_{\text{ref}}$, dès que la trajectoire théorique incurve sa course, le drone s'incline *instantanément* de l'angle exact requis pour suivre la courbe sans erreur de traînage.

---

## 3. Inversion du PFD et Projection dans le Body : Trouver $T_d, \phi_d, \theta_d$

C'est **l'étape mathématique centrale** : comment traduire l'accélération terrestre souhaitée $\vec{a}_{\text{des}} = (a_{x,\text{des}}, a_{y,\text{des}}, a_{z,\text{des}})^T$ en une poussée moteur $T_d$ et deux angles d'inclinaison $(\phi_d, \theta_d)$ ?

### A. Écriture du PFD dans le repère NED
Appliquons la 2ème loi de Newton (PFD) au centre de gravité du drone dans le repère inertiel NED :

$$
m \vec{a}_{\text{des}} = m \vec{g}_I + R_{B \to I}(\phi_d, \theta_d, \psi) \vec{F}_{\text{poussée}, B}
$$

En remplaçant les vecteurs par leurs coordonnées respectives en NED et FRD :

$$
m \begin{pmatrix} a_{x,\text{des}} \\ a_{y,\text{des}} \\ a_{z,\text{des}} \end{pmatrix} = \begin{pmatrix} 0 \\ 0 \\ +mg \end{pmatrix} + R_z(\psi) R_y(\theta_d) R_x(\phi_d) \begin{pmatrix} 0 \\ 0 \\ -T_d \end{pmatrix}
$$

Divisons toute l'équation par la masse $m$ et passons la gravité à gauche du signe égal :

$$
\begin{pmatrix} a_{x,\text{des}} \\ a_{y,\text{des}} \\ a_{z,\text{des}} - g \end{pmatrix}_{\text{NED}} = R_z(\psi) R_y(\theta_d) R_x(\phi_d) \begin{pmatrix} 0 \\ 0 \\ -\frac{T_d}{m} \end{pmatrix}
$$

> **Observation clé (Feedforward de gravité) :** Le terme $-g$ qui apparaît sur l'axe $Z$ est la **compensation statique de gravité** (*Gravity Feedforward*). Si le drone doit rester en vol stationnaire ($a_{z,\text{des}} = 0$), les moteurs doivent produire une accélération propre égale à $-g$ (vers le haut) pour vaincre le poids $+g$ (vers le bas).

### B. Pourquoi et Comment projeter pour éliminer le Lacet ($\psi$) ?

**Le problème géométrique :**
Les accélérations $a_{x,\text{des}}$ et $a_{y,\text{des}}$ sont exprimées vers le **Nord** et l'**Est**. Or, les angles de tangage ($\theta$) et de roulis ($\phi$) inclinent le drone vers **son Avant** et **sa Droite**.
Si le drone a tourné sur lui-même d'un angle de lacet $\psi$, son nez ne pointe plus vers le Nord !
Il faut donc **projeter** le vecteur d'accélération désirée dans un **repère intermédiaire horizontal aligné avec le nez du drone** (obtenu en tournant le repère NED d'un angle $\psi$ autour de $Z$).

**L'opération mathématique :**
On multiplie l'équation du PFD à gauche par l'inverse de la matrice de lacet, soit $R_z(\psi)^{-1} = R_z(\psi)^T = R_z(-\psi)$ :

$$
\underbrace{\begin{pmatrix} \cos\psi & \sin\psi & 0 \\ -\sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{pmatrix}}_{R_z(-\psi)} \begin{pmatrix} a_{x,\text{des}} \\ a_{y,\text{des}} \\ a_{z,\text{des}} - g \end{pmatrix} = R_y(\theta_d) R_x(\phi_d) \begin{pmatrix} 0 \\ 0 \\ -\frac{T_d}{m} \end{pmatrix}
$$

Calculons le vecteur de gauche, qui représente désormais **les accélérations désirées dans l'axe du châssis du drone** :
* **Accélération longitudinale désirée (vers l'avant du nez) :**
  $$
  \tilde{a}_x = a_{x,\text{des}} \cos\psi + a_{y,\text{des}} \sin\psi
  $$
* **Accélération latérale désirée (vers le flanc droit) :**
  $$
  \tilde{a}_y = -a_{x,\text{des}} \sin\psi + a_{y,\text{des}} \cos\psi
  $$
* **Accélération verticale désirée (compensée de la gravité) :**
  $$
  \tilde{a}_z = a_{z,\text{des}} - g
  $$

Calculons maintenant le membre de droite en multipliant $R_y(\theta_d) R_x(\phi_d)$ par $\begin{pmatrix} 0 & 0 & -T_d/m \end{pmatrix}^T$ :

$$
\begin{pmatrix} \cos\theta_d & \sin\theta_d\sin\phi_d & \sin\theta_d\cos\phi_d \\ 0 & \cos\phi_d & -\sin\phi_d \\ -\sin\theta_d & \cos\theta_d\sin\phi_d & \cos\theta_d\cos\phi_d \end{pmatrix} \begin{pmatrix} 0 \\ 0 \\ -\frac{T_d}{m} \end{pmatrix} = \begin{pmatrix} -\frac{T_d}{m} \sin\theta_d \cos\phi_d \\ +\frac{T_d}{m} \sin\phi_d \\ -\frac{T_d}{m} \cos\theta_d \cos\phi_d \end{pmatrix}
$$

En égalant la gauche et la droite, on obtient le **Système Fondamental Découplé (NED/FRD)** :

$$
\begin{cases}
\tilde{a}_x = -\frac{T_d}{m} \sin\theta_d \cos\phi_d & \text{(Éq. 1 : Axe Avant)} \\[6pt]
\tilde{a}_y = +\frac{T_d}{m} \sin\phi_d & \text{(Éq. 2 : Axe Droite)} \\[6pt]
a_{z,\text{des}} - g = -\frac{T_d}{m} \cos\theta_d \cos\phi_d & \text{(Éq. 3 : Axe Bas)}
\end{cases}
$$

### C. Résolution : Expressions Exactes et Petits Angles

À partir de ces 3 équations ultra-propres, on extrait nos 3 inconnues $(T_d, \theta_d, \phi_d)$ :

#### 1. Calcul de la Poussée Totale Désirée $T_d$ (Commande $U_1$)
* **Formule exacte (Norme euclidienne) :** En élevant les 3 équations au carré et en les sommant ($\sin^2 + \cos^2 = 1$) :
  $$
  T_d = m \sqrt{\tilde{a}_x^2 + \tilde{a}_y^2 + (a_{z,\text{des}} - g)^2} = m \sqrt{a_{x,\text{des}}^2 + a_{y,\text{des}}^2 + (a_{z,\text{des}} - g)^2}
  $$
* **Formule par projection verticale (compensation d'inclinaison) :** En isolant $T_d$ dans l'Éq. 3 :
  $$
  T_d = \frac{m (g - a_{z,\text{des}})}{\cos\theta \cos\phi}
  $$
* **Approximation aux petits angles ($\phi \approx 0, \theta \approx 0$) :**
  $$
  \boxed{T_d \approx m (g - a_{z,\text{des}})}
  $$

#### 2. Calcul de l'Angle de Tangage Désiré $\theta_d$ (*Pitch*)
* **Formule exacte (Non-linéaire) :** On divise l'Éq. 1 par l'Éq. 3. Les termes $-\frac{T_d}{m}$ et $\cos\phi_d$ s'éliminent !
  $$
  \frac{\tilde{a}_x}{a_{z,\text{des}} - g} = \frac{-\frac{T_d}{m} \sin\theta_d \cos\phi_d}{-\frac{T_d}{m} \cos\theta_d \cos\phi_d} = \tan\theta_d \quad \Longrightarrow \quad \boxed{\theta_d = \arctan\left( \frac{a_{x,\text{des}} \cos\psi + a_{y,\text{des}} \sin\psi}{a_{z,\text{des}} - g} \right)}
  $$
* **Approximation aux petits angles ($a_{z,\text{des}} \ll g$ et $\tan\theta_d \approx \theta_d$) :**
  $$
  \boxed{\theta_d \approx -\frac{\tilde{a}_x}{g} = -\frac{1}{g} \left( a_{x,\text{des}} \cos\psi + a_{y,\text{des}} \sin\psi \right)}
  $$
  *(Vérification physique : Si $\psi=0$ et qu'on veut avancer vers le Nord $a_{x,\text{des}} > 0$, alors $\theta_d < 0$, le drone pique bien du nez !)*

#### 3. Calcul de l'Angle de Roulis Désiré $\phi_d$ (*Roll*)
* **Formule exacte (Non-linéaire) :** On isole $\sin\phi_d$ dans l'Éq. 2 :
  $$
  \sin\phi_d = \frac{m \tilde{a}_y}{T_d} \quad \Longrightarrow \quad \boxed{\phi_d = \arcsin\left( \frac{-a_{x,\text{des}} \sin\psi + a_{y,\text{des}} \cos\psi}{\sqrt{a_{x,\text{des}}^2 + a_{y,\text{des}}^2 + (a_{z,\text{des}} - g)^2}} \right)}
  $$
  *(Alternative par l'Éq. 3 : $\phi_d = \arctan\left( \frac{\tilde{a}_y \cos\theta_d}{g - a_{z,\text{des}}} \right)$).*
* **Approximation aux petits angles ($T_d \approx mg$ et $\sin\phi_d \approx \phi_d$) :**
  $$
  \boxed{\phi_d \approx +\frac{\tilde{a}_y}{g} = \frac{1}{g} \left( -a_{x,\text{des}} \sin\psi + a_{y,\text{des}} \cos\psi \right)}
  $$
  *(Vérification physique : Si $\psi=0$ et qu'on veut aller vers l'Est/Droite $a_{y,\text{des}} > 0$, alors $\phi_d > 0$, le drone s'incline bien à droite !)*

---

## 4. TMD (Rotation) et Boucle Interne d'Attitude : PID + Feedforward

Maintenant que nous avons les angles désirés $(\phi_d, \theta_d)$ et le lacet désiré $\psi_d$, la boucle interne d'attitude doit générer les **couples moteurs** $\vec{\tau}_m = (\tau_\phi, \tau_\theta, \tau_\psi)^T$ dans le repère Body **FRD**.

### A. Le Théorème du Moment Dynamique (TMD) en repère FRD
Soit $\vec{\omega} = (p, q, r)^T$ le vecteur vitesse de rotation instantanée du drone dans le repère Body $\mathcal{R}_B$ (mesuré par les gyroscopes) et $\mathbf{J} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ sa matrice d'inertie.
Par dérivation dans un repère mobile (formule de Bour), le TMD s'écrit :

$$
\mathbf{J} \dot{\vec{\omega}} + \vec{\omega} \wedge (\mathbf{J} \vec{\omega}) = \vec{\tau}_m
$$

En développant le produit vectoriel gyroscopique $\vec{\omega} \wedge (\mathbf{J}\vec{\omega})$, cela donne :

$$
\begin{cases}
I_{xx} \dot{p} + (I_{zz} - I_{yy}) q r = \tau_\phi \\
I_{yy} \dot{q} + (I_{xx} - I_{zz}) p r = \tau_\theta \\
I_{zz} \dot{r} + (I_{yy} - I_{xx}) p q = \tau_\psi
\end{cases}
$$

### B. Loi de Commande d'Attitude Complète (Feedback PID + Feedforward)
Pour obtenir un suivi d'attitude ultra-rapide et linéariser l'équation du TMD (*Computed Torque Control*), on calcule les couples moteurs $\vec{\tau}_m$ en additionnant :
1. **Le Feedback PID** sur l'erreur d'angle $(\phi_d - \phi)$ et de vitesse angulaire $(p_d - p)$.
2. **Le Feedforward d'accélération angulaire** $\mathbf{J} \dot{\vec{\omega}}_d$ (pour vaincre l'inertie instantanément lors d'un changement d'inclinaison).
3. **Le Feedforward de découplage gyroscopique** $\vec{\omega} \wedge (\mathbf{J} \vec{\omega})$ (qui compense l'effet de Coriolis).

$$
\begin{pmatrix} \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{pmatrix} = \underbrace{\boldsymbol{K_{p,\text{att}}} \begin{pmatrix} \phi_d - \phi \\ \theta_d - \theta \\ \psi_d - \psi \end{pmatrix} + \boldsymbol{K_{d,\text{att}}} \begin{pmatrix} p_d - p \\ q_d - q \\ r_d - r \end{pmatrix} + \boldsymbol{K_{i,\text{att}}} \int \begin{pmatrix} \phi_d - \phi \\ \theta_d - \theta \\ \psi_d - \psi \end{pmatrix} dt}_{\text{Feedback PID d'Attitude}} + \underbrace{\begin{pmatrix} I_{xx} \dot{p}_d \\ I_{yy} \dot{q}_d \\ I_{zz} \dot{r}_d \end{pmatrix}}_{\substack{\text{Feedforward} \\ \text{Inertiel}}} + \underbrace{\begin{pmatrix} (I_{zz} - I_{yy}) q r \\ (I_{xx} - I_{zz}) p r \\ (I_{yy} - I_{xx}) p q \end{pmatrix}}_{\substack{\text{Feedforward} \\ \text{Gyroscopique}}}
$$

*(Note : Aux petits angles, on approxime souvent les vitesses angulaires de consigne par $(p_d, q_d, r_d) \approx (\dot{\phi}_d, \dot{\theta}_d, \dot{\psi}_d)$).*

---

## 5. Mixage Moteur (Control Allocation) en Repère FRD

Nous disposons maintenant du vecteur de commande complet $U = (T_d, \tau_\phi, \tau_\theta, \tau_\psi)^T$. Il faut le convertir en 4 forces de poussée individuelles $(f_1, f_2, f_3, f_4)$ pour chacun des 4 moteurs.

### A. Modèle d'un Moteur
Chaque hélice $i \in \{1, 2, 3, 4\}$ tournant à la vitesse angulaire $\omega_i$ (en rad/s) génère :
* Une **force de poussée** dirigée vers le haut ($-\vec{z}_B$) : $f_i = k_f \, \omega_i^2$
* Un **couple de traînée aérodynamique** autour de $\vec{z}_B$ : $\tau_{\text{aero}, i} = \pm k_\tau \, \omega_i^2 = \pm c_\tau \, f_i$ (où $c_\tau = \frac{k_\tau}{k_f}$).

### B. Bilan des Moments en FRD (Cas 1 : Configuration en "X" — Standard PX4)
Soit un drone en configuration **"X"** dont les moteurs sont situés à une distance $l = L \frac{\sqrt{2}}{2}$ des axes $\vec{x}_B$ et $\vec{y}_B$ :

```
          AVANT (+x_B)
       M3 (CW)    M1 (CCW)
          \        /
           \      /
  (-y_B)    [ G ]     (+y_B) DROITE
  GAUCHE   /      \
          /        \
       M2 (CCW)   M4 (CW)
         ARRIÈRE (-x_B)
```

* **Moteur 1 (Avant-Droit)** : $(+l, +l)$, tourne en sens **Anti-Horaire (CCW)** vu du dessus.
* **Moteur 2 (Arrière-Gauche)** : $(-l, -l)$, tourne en sens **Anti-Horaire (CCW)**.
* **Moteur 3 (Avant-Gauche)** : $(+l, -l)$, tourne en sens **Horaire (CW)**.
* **Moteur 4 (Arrière-Droit)** : $(-l, +l)$, tourne en sens **Horaire (CW)**.

**Établissons les 4 équations physiques dans le repère FRD (avec la règle de la main droite) :**

1. **Poussée Totale ($T_d$) :** Somme des 4 forces.
   $$
   T_d = f_1 + f_2 + f_3 + f_4
   $$
2. **Couple de Roulis ($\tau_\phi$ autour de $+\vec{x}_B$) :** Un couple $\tau_\phi > 0$ abaisse la droite ($+y_B$) et lève la gauche ($-y_B$). Les moteurs de **Gauche (M2, M3)** ont donc une contribution **positive**, et ceux de **Droite (M1, M4)** une contribution **négative** :
   $$
   \tau_\phi = l ( -f_1 + f_2 + f_3 - f_4 )
   $$
3. **Couple de Tangage ($\tau_\theta$ autour de $+\vec{y}_B$) :** Un couple $\tau_\theta > 0$ cabre le drone : il lève l'avant ($+x_B$) et abaisse l'arrière ($-x_B$). Les moteurs **Avant (M1, M3)** ont donc une contribution **positive**, et les moteurs **Arrière (M2, M4)** une contribution **négative** :
   $$
   \tau_\theta = l ( +f_1 - f_2 + f_3 - f_4 )
   $$
4. **Couple de Lacet ($\tau_\psi$ autour de $+\vec{z}_B$) :** Un couple $\tau_\psi > 0$ fait tourner le nez vers la droite (sens **Horaire / CW** vu du dessus). Par action/réaction avec l'air, ce sont les hélices qui tournent dans le sens **Anti-Horaire (CCW : M1, M2)** qui créent un couple réactif **Horaire ($+\tau_\psi$)** sur le châssis :
   $$
   \tau_\psi = c_\tau ( +f_1 + f_2 - f_3 - f_4 )
   $$

### C. Matrice d'Allocation et Inversion (Consignes Finales aux Moteurs)

Sous forme matricielle :
$$
\begin{pmatrix} T_d \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{pmatrix} = \begin{pmatrix} 1 & 1 & 1 & 1 \\ -l & l & l & -l \\ l & -l & l & -l \\ c_\tau & c_\tau & -c_\tau & -c_\tau \end{pmatrix} \begin{pmatrix} f_1 \\ f_2 \\ f_3 \\ f_4 \end{pmatrix}
$$

En inversant cette matrice de mixage, on obtient les **4 forces de poussée finales** à commander :

$$
\boxed{
\begin{cases}
f_1 \text{ (Avant-Droit, CCW)} &= \frac{1}{4} T_d - \frac{1}{4l} \tau_\phi + \frac{1}{4l} \tau_\theta + \frac{1}{4 c_\tau} \tau_\psi \\[8pt]
f_2 \text{ (Arrière-Gauche, CCW)} &= \frac{1}{4} T_d + \frac{1}{4l} \tau_\phi - \frac{1}{4l} \tau_\theta + \frac{1}{4 c_\tau} \tau_\psi \\[8pt]
f_3 \text{ (Avant-Gauche, CW)} &= \frac{1}{4} T_d + \frac{1}{4l} \tau_\phi + \frac{1}{4l} \tau_\theta - \frac{1}{4 c_\tau} \tau_\psi \\[8pt]
f_4 \text{ (Arrière-Droit, CW)} &= \frac{1}{4} T_d - \frac{1}{4l} \tau_\phi - \frac{1}{4l} \tau_\theta - \frac{1}{4 c_\tau} \tau_\psi
\end{cases}
}
$$

*(Note pour une **configuration en "+"** en FRD avec M1 Devant, M2 Droite, M3 Arrière, M4 Gauche et bras de longueur $L$ : on a simplement $\tau_\phi = L(f_4 - f_2)$ et $\tau_\theta = L(f_1 - f_3)$).*

### D. Où se trouvent les Feedforwards dans les équations moteurs finales ?
Si l'on remplace $T_d$ (aux petits angles) dans l'expression de $f_i$, on voit apparaître explicitement toute la chaîne d'anticipation et de rétroaction :

$$
f_i = \underbrace{\frac{m g}{4}}_{\substack{\text{Feedforward} \\ \text{Poids (Hover)}}} - \underbrace{\frac{m}{4} a_{z,\text{ref}}}_{\substack{\text{Feedforward} \\ \text{Trajectoire Z}}} - \underbrace{\frac{m}{4} \text{PID}_z(z_{\text{ref}} - z)}_{\text{Feedback Altitude}} \pm \frac{\tau_\phi(\text{PID} + \text{FF})}{4l} \pm \frac{\tau_\theta(\text{PID} + \text{FF})}{4l} \pm \frac{\tau_\psi(\text{PID} + \text{FF})}{4c_\tau}
$$

Enfin, chaque force $f_i$ est convertie en consigne de vitesse de rotation envoyée aux ESC (contrôleurs moteurs) :
$$
\omega_{i, \text{cmd}} = \sqrt{\frac{\max(0, \, f_i)}{k_f}}
$$

---

## 6. Schéma Récapitulatif Global de l'Architecture (NED / FRD + FF)

```
               [TRAJECTOIRE DE RÉFÉRENCE EN NED]
          Position p_ref(t) | Vitesse v_ref(t) | Accélération a_ref(t) (FF)
                            │
                            ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 1. CONTRÔLEUR DE POSITION (Repère NED)                                    │
│    a_des = PID_pos(p_ref - p, v_ref - v) + a_ref (Feedforward Trajectoire)│
└───────────────────────────────────────────────────────────────────────────┘
                            │
                            ▼  (Accélérations désirées : a_x,des, a_y,des, a_z,des)
┌───────────────────────────────────────────────────────────────────────────┐
│ 2. PROJECTION BODY & INVERSION DU PFD (Passage NED ──► FRD)               │
│                                                                           │
│  • Rotation de Lacet (-ψ) pour aligner avec le nez du drone :             │
│      ã_x (Avant)  =  a_x,des * cos(ψ) + a_y,des * sin(ψ)                  │
│      ã_y (Droite) = -a_x,des * sin(ψ) + a_y,des * cos(ψ)                  │
│                                                                           │
│  • Extraction de la Poussée (avec Feedforward de Gravité -g) & Angles :   │
│      T_d = m * sqrt(ã_x² + ã_y² + (a_z,des - g)²)  ≈ m * (g - a_z,des)    │
│      θ_d = arctan( ã_x / (a_z,des - g) )           ≈ -ã_x / g  (Pitch)    │
│      φ_d = arcsin( m * ã_y / T_d )                 ≈ +ã_y / g  (Roll)     │
└───────────────────────────────────────────────────────────────────────────┘
                            │
       ┌────────────────────┴────────────────────┐
       │ Poussée Totale T_d                      │ Angles désirés (φ_d, θ_d, ψ_d)
       │                                         ▼
       │    ┌───────────────────────────────────────────────────────────────┐
       │    │ 3. CONTRÔLEUR D'ATTITUDE (Repère FRD - Inversion TMD)         │
       │    │    τ_m = PID_att(θ_d - θ, ω_d - ω)                            │
       │    │          + J * dω_d/dt         (Feedforward Accél. Angulaire) │
       │    │          + ω ∧ (J * ω)         (Feedforward Gyroscopique)     │
       │    └───────────────────────────────────────────────────────────────┘
       │                                         │
       │                                         ▼ Couples (τ_φ, τ_θ, τ_ψ)
       └────────────────────┬────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────────────────┐
│ 4. MIXEUR MOTEUR (Control Allocation en FRD - Config X)                   │
│    f_1 = T_d/4 - τ_φ/(4l) + τ_θ/(4l) + τ_ψ/(4c_τ)                         │
│    f_2 = T_d/4 + τ_φ/(4l) - τ_θ/(4l) + τ_ψ/(4c_τ)                         │
│    f_3 = T_d/4 + τ_φ/(4l) + τ_θ/(4l) - τ_ψ/(4c_τ)                         │
│    f_4 = T_d/4 - τ_φ/(4l) - τ_θ/(4l) - τ_ψ/(4c_τ)                         │
└───────────────────────────────────────────────────────────────────────────┘
                            │
                            ▼
       [Vitesses de rotation des 4 moteurs : ω_i = sqrt(f_i / k_f)]
```
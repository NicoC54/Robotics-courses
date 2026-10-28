Voici ta fiche de cours magistrale. Elle est rédigée de bout en bout avec les notations strictes **$W$ (*World* = NED)** et **$B$ (*Body* = FRD)**, en détaillant systématiquement **le besoin d'ingénierie**, **le théorème mécanique invoqué**, **la démonstration algébrique ligne par ligne** et **l'implication directe pour un Filtre de Kalman Étendu (EKF)**.

---

# COURS COMPLET : MODÉLISATION DYNAMIQUE 6-DDL DES DRONES MULTIROTORS (CONVENTIONS $W$-NED / $B$-FRD) & PRÉPARATION À L'EKF

---

## MODULE 0 : Philosophie Générale — Pourquoi modélise-t-on dans deux repères différents ?

### 1. Le dilemme fondamental de l'ingénieur robotique

Un drone évolue dans l'espace mais ses capteurs et ses actionneurs sont solidaires de son châssis. On fait face à une contradiction physique :

* **L'objectif de navigation et la gravité sont liés à la Terre ($W$) :** Le GPS mesure une position dans $W$, la consigne de vol est donnée dans $W$, et le champ de pesanteur terrestre $\mathbf{g}^W$ tire verticalement vers le centre de la Terre, peu importe l'inclinaison du drone.
* **La propulsion, l'inertie et l'IMU sont liées au châssis ($B$) :** Les hélices sont boulonnées sur les bras du drone et poussent toujours perpendiculairement au châssis (axe $-z_B$). Le gyroscope et l'accéléromètre (IMU) sont soudés sur la carte électronique et mesurent les vitesses angulaires et accélérations spécifiques dans $B$. Surtout, **la matrice d'inertie $\mathbf{I}_B$ est constante dans $B$**, alors qu'elle dépendrait du temps dans $W$.

> **Règle d'or de la modélisation :** On écrit toujours une loi physique dans le repère où ses grandeurs géométriques (bras de levier, matrice d'inertie) sont **constantes**, puis on utilise des matrices de rotation pour transporter les vecteurs d'un repère à l'autre.

### 2. Définition rigoureuse des repères $W$ (NED) et $B$ (FRD)

Tous les repères utilisés sont **orthonormés directs** (règle de la main droite : Pouce = $x$, Index = $y$, Majeur = $z$ ; et pour le sens positif d'une rotation : on met le pouce de la main droite dans le sens de l'axe, les doigts qui s'enroulent donnent le sens positif de l'angle).

* **Repère Monde $\mathcal{R}_W (O_W, \mathbf{x}_W, \mathbf{y}_W, \mathbf{z}_W)$ — Convention NED (*North-East-Down*) :**
* $\mathbf{x}_W$ : pointe vers le Nord géographique.
* $\mathbf{y}_W$ : pointe vers l'Est géographique.
* $\mathbf{z}_W$ : pointe vers le bas (suivant le fil à plomb, vers le centre de la Terre).
* *Piège à retenir :* Une altitude de $15\text{ m}$ au-dessus du sol s'écrit $z_W = -15\text{ m}$. Monter correspond à $\dot{z}_W$ négatif. Le vecteur accélération de la pesanteur s'écrit $\mathbf{g}^W = [0, 0, +g]^T$ avec $g \approx 9.81\text{ m/s}^2$.


* **Repère Corps $\mathcal{R}_B (G, \mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B)$ — Convention FRD (*Forward-Right-Down*) :**
* Origine $G$ : centre de masse (centre de gravité) du drone.
* $\mathbf{x}_B$ : pointe vers l'avant (nez du drone).
* $\mathbf{y}_B$ : pointe vers le flanc droit.
* $\mathbf{z}_B$ : pointe vers le ventre du drone.
* *Piège à retenir :* Pour voler, les hélices soufflent l'air vers le bas ($+\mathbf{z}_B$) et créent donc par réaction une force de portance dirigée vers le dos du drone, soit selon $-\mathbf{z}_B$.



### 3. Convention de notation matricielle (Standard Robotique)

Pour ne jamais se tromper dans les produits matriciels :

* Un vecteur géométrique $\mathbf{v}$ exprimé dans le repère $W$ est noté $\mathbf{v}^W$. Exprimé dans $B$, il est noté $\mathbf{v}^B$.
* La matrice de rotation qui transforme les coordonnées d'un vecteur depuis le repère $B$ vers le repère $W$ est notée $\mathbf{R}_B^W$.
* **Règle d'annulation des indices en diagonale :**

$$\mathbf{v}^W = \mathbf{R}_B^W \, \mathbf{v}^B$$



*(L'indice $B$ en bas à droite de $\mathbf{R}_B^W$ « mange » l'exposant $B$ de $\mathbf{v}^B$, il ne reste que $W$).*
* Comme $\mathbf{R}_B^W$ est une matrice orthogonale ($\mathbf{R} \in SO(3)$), son inverse est tout simplement sa transposée :

$$\mathbf{R}_W^B = \left(\mathbf{R}_B^W\right)^{-1} = \left(\mathbf{R}_B^W\right)^T$$



---

## MODULE 1 : Cinématique d'Orientation — Comment passer de $B$ à $W$ ?

### 1. Pourquoi fait-on cette étape maintenant ?

Avant même de parler de forces ou de moteurs, il faut construire l'opérateur mathématique $\mathbf{R}_B^W$ qui permettra de projeter la poussée des moteurs (connue dans $B$) vers le repère terrestre $W$, ou de projeter la gravité (connue dans $W$) vers les capteurs du drone ($B$).

### 2. Quelle formule / méthode applique-t-on et pourquoi ?

En 3D, les rotations ne commutent pas. Pour décrire l'orientation de $B$ par rapport à $W$, la norme aéronautique utilise les **angles de Tait-Bryan (souvent appelés angles d'Euler) selon la séquence intrinsèque Z-Y-X** :

1. On part du repère $W$. On tourne d'un angle de **Lacet (*Yaw*) $\psi$** autour de $\mathbf{z}_W$. On obtient un premier repère intermédiaire $\mathcal{R}_1$.
2. On tourne d'un angle de **Tangage (*Pitch*) $\theta$** autour du nouvel axe $\mathbf{y}_1$. On obtient un second repère intermédiaire $\mathcal{R}_2$.
3. On tourne d'un angle de **Roulis (*Roll*) $\phi$** autour du nouvel axe $\mathbf{x}_2$. On arrive sur le repère final du drone $\mathcal{R}_B$.

Par la règle de chaînage des indices :


$$\mathbf{R}_B^W = \mathbf{R}_1^W(\psi) \cdot \mathbf{R}_2^1(\theta) \cdot \mathbf{R}_B^2(\phi)$$

### 3. Comment construit-on chacune des 3 matrices élémentaires $3 \times 3$ ?

**Méthode universelle :** Les colonnes d'une matrice de passage $\mathbf{R}_{\text{nouveau}}^{\text{ancien}}$ sont les coordonnées des vecteurs unitaires du **nouveau** repère projetées dans l'**ancien** repère.

* **Rotation 1 : Lacet $\psi$ autour de $\mathbf{z}_W$ ($\mathcal{R}_W \to \mathcal{R}_1$)**
Puisqu'on tourne autour de $\mathbf{z}_W$, l'axe $\mathbf{z}_1$ reste identique à $\mathbf{z}_W$ : sa colonne vaut $[0, 0, 1]^T$.
Dans le plan horizontal $(\mathbf{x}_W, \mathbf{y}_W)$, quand on tourne $\mathbf{x}_1$ d'un angle $+\psi$ (du Nord vers l'Est), sa projection sur $\mathbf{x}_W$ est $\cos\psi$ et sur $\mathbf{y}_W$ est $+\sin\psi$.
Quant à l'axe $\mathbf{y}_1$, il avance dans le quadrant $(-\mathbf{x}_W, +\mathbf{y}_W)$ : sa projection sur $\mathbf{x}_W$ est $-\sin\psi$ et sur $\mathbf{y}_W$ est $\cos\psi$.

$$\mathbf{R}_1^W(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$


* **Rotation 2 : Tangage $\theta$ autour de $\mathbf{y}_1$ ($\mathcal{R}_1 \to \mathcal{R}_2$)**
Puisqu'on tourne autour de $\mathbf{y}_1$, la 2ème colonne et la 2ème ligne valent $[0, 1, 0]$.
*Attention au signe (piège classique) :* Par la règle de la main droite, mettons le pouce le long de $\mathbf{y}_1$ (vers la droite). Nos doigts s'enroulent en faisant **monter le nez $\mathbf{x}_2$ vers le haut** (donc vers $-\mathbf{z}_1$, puisque $\mathbf{z}_1$ pointe vers le bas !).
Donc, pour un angle $\theta > 0$, la projection de $\mathbf{x}_2$ sur $\mathbf{x}_1$ est $\cos\theta$, et sa projection sur $\mathbf{z}_1$ est **$-\sin\theta$**.
De même, l'axe $\mathbf{z}_2$ bascule légèrement vers l'avant ($+\mathbf{x}_1$), donc sa projection sur $\mathbf{x}_1$ est $+\sin\theta$ et sur $\mathbf{z}_1$ est $\cos\theta$.

$$\mathbf{R}_2^1(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$


* **Rotation 3 : Roulis $\phi$ autour de $\mathbf{x}_2$ ($\mathcal{R}_2 \to \mathcal{R}_B$)**
Puisqu'on tourne autour de $\mathbf{x}_2$, la 1ère ligne et la 1ère colonne valent $[1, 0, 0]$.
Mettons le pouce vers l'avant ($\mathbf{x}_2$) : les doigts enfoncent l'aile droite $\mathbf{y}_B$ vers le bas ($+\mathbf{z}_2$). Donc $\mathbf{y}_B$ se projette en $\cos\phi$ sur $\mathbf{y}_2$ et $+\sin\phi$ sur $\mathbf{z}_2$.

$$\mathbf{R}_B^2(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}$$



### 4. Multiplication matricielle détaillée pour obtenir $\mathbf{R}_B^W$

Notons $c_\alpha = \cos\alpha$ et $s_\alpha = \sin\alpha$. Calculons d'abord le produit des deux dernières matrices $\mathbf{R}_2^1(\theta) \cdot \mathbf{R}_B^2(\phi)$ :

$$\mathbf{R}_2^1(\theta) \cdot \mathbf{R}_B^2(\phi) = \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\phi & -s_\phi \\ 0 & s_\phi & c_\phi \end{bmatrix} = \begin{bmatrix} c_\theta & s_\theta s_\phi & s_\theta c_\phi \\ 0 & c_\phi & -s_\phi \\ -s_\theta & c_\theta s_\phi & c_\theta c_\phi \end{bmatrix}$$

Multiplions maintenant à gauche par $\mathbf{R}_1^W(\psi)$ :

$$\mathbf{R}_B^W = \begin{bmatrix} c_\psi & -s_\psi & 0 \\ s_\psi & c_\psi & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} c_\theta & s_\theta s_\phi & s_\theta c_\phi \\ 0 & c_\phi & -s_\phi \\ -s_\theta & c_\theta s_\phi & c_\theta c_\phi \end{bmatrix}$$

On obtient la **Matrice de Rotation Directe $\mathbf{R}_B^W$ (de Body vers World)** :

$$\mathbf{R}_B^W = \begin{bmatrix} c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \\ s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \\ -s_\theta & c_\theta s_\phi & c_\theta c_\phi \end{bmatrix}$$

Et par simple transposition (les lignes deviennent les colonnes), la **Matrice de Rotation Inverse $\mathbf{R}_W^B$ (de World vers Body)** :

$$\mathbf{R}_W^B = \left(\mathbf{R}_B^W\right)^T = \begin{bmatrix} c_\psi c_\theta & s_\psi c_\theta & -s_\theta \\ c_\psi s_\theta s_\phi - s_\psi c_\phi & s_\psi s_\theta s_\phi + c_\psi c_\phi & c_\theta s_\phi \\ c_\psi s_\theta c_\phi + s_\psi s_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi & c_\theta c_\phi \end{bmatrix}$$

---

## MODULE 2 : Cinématique de Rotation — Relier le Gyroscope ($\boldsymbol{\omega}^B$) aux Angles d'Euler ($\dot{\boldsymbol{\eta}}$)

### 1. Pourquoi fait-on cette étape ?

La centrale inertielle (IMU) du drone contient un gyroscope 3 axes qui mesure le vecteur vitesse de rotation instantanée du corps exprimé dans $B$ :


$$\boldsymbol{\omega}^B = \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$


Or, pour connaître l'orientation du drone à l'instant suivant (et mettre à jour $\mathbf{R}_B^W$), nous devons intégrer les dérivées des angles d'Euler :


$$\dot{\boldsymbol{\eta}} = \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}$$


**Attention :** $\boldsymbol{\omega}^B \neq \dot{\boldsymbol{\eta}}$ dès que le drone est incliné ! Il nous faut la matrice de transformation exacte entre les deux.

### 2. Quelle loi applique-t-on et pourquoi ?

On applique la **loi de composition des vecteurs vitesses de rotation** : la vitesse angulaire totale du repère $B$ par rapport à $W$ est la somme vectorielle des vitesses angulaires des 3 rotations intermédiaires :


$$\boldsymbol{\omega}_{B/W} = \boldsymbol{\omega}_{B/2} + \boldsymbol{\omega}_{2/1} + \boldsymbol{\omega}_{1/W}$$


Mais pour additionner ces trois vecteurs dans une équation matricielle, **il faut les exprimer tous les trois dans le même repère final $\mathcal{R}_B$** !

* $\boldsymbol{\omega}_{B/2} = [\dot{\phi}, 0, 0]^T$ est *déjà* exprimé dans $\mathcal{R}_B$ (car l'axe de roulis $\mathbf{x}_2$ coïncide avec $\mathbf{x}_B$).
* $\boldsymbol{\omega}_{2/1} = [0, \dot{\theta}, 0]^T$ est exprimé dans $\mathcal{R}_2$. Pour l'amener dans $\mathcal{R}_B$, il faut lui faire subir l'inverse de la dernière rotation (le roulis $\phi$), soit multiplier par $\mathbf{R}_2^B(\phi) = \left(\mathbf{R}_B^2(\phi)\right)^T$.
* $\boldsymbol{\omega}_{1/W} = [0, 0, \dot{\psi}]^T$ est exprimé dans $\mathcal{R}_1$. Pour l'amener dans $\mathcal{R}_B$, il faut lui faire subir successivement l'inverse du tangage $\theta$ puis l'inverse du roulis $\phi$, soit multiplier par $\mathbf{R}_2^B(\phi) \cdot \mathbf{R}_1^2(\theta)$.

### 3. Développement algébrique ligne par ligne

Écrivons la somme projetée dans $\mathcal{R}_B$ :


$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} \dot{\phi} \\ 0 \\ 0 \end{bmatrix} + \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\phi & s_\phi \\ 0 & -s_\phi & c_\phi \end{bmatrix} \begin{bmatrix} 0 \\ \dot{\theta} \\ 0 \end{bmatrix} + \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\phi & s_\phi \\ 0 & -s_\phi & c_\phi \end{bmatrix} \begin{bmatrix} c_\theta & 0 & -s_\theta \\ 0 & 1 & 0 \\ s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix}$$

Calculons chaque terme :

1. Deuxième terme : $\begin{bmatrix} 0 \\ \dot{\theta} c_\phi \\ -\dot{\theta} s_\phi \end{bmatrix}$
2. Troisième terme : d'abord $\mathbf{R}_1^2(\theta) \begin{bmatrix} 0 \\ 0 \\ \dot{\psi} \end{bmatrix} = \begin{bmatrix} -\dot{\psi} s_\theta \\ 0 \\ \dot{\psi} c_\theta \end{bmatrix}$, puis multiplié par $\mathbf{R}_2^B(\phi)$ donne $\begin{bmatrix} -\dot{\psi} s_\theta \\ \dot{\psi} s_\phi c_\theta \\ \dot{\psi} c_\phi c_\theta \end{bmatrix}$.

Regroupons sous forme matricielle $\boldsymbol{\omega}^B = \mathbf{W}_\eta \, \dot{\boldsymbol{\eta}}$ :


$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} = \underbrace{\begin{bmatrix} 1 & 0 & -s_\theta \\ 0 & c_\phi & s_\phi c_\theta \\ 0 & -s_\phi & c_\phi c_\theta \end{bmatrix}}_{\mathbf{W}_\eta} \begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix}$$

Dans un simulateur ou un EKF, on connaît $[p, q, r]^T$ (lu par le gyroscope) et on cherche $[\dot{\phi}, \dot{\theta}, \dot{\psi}]^T$. On inverse donc $\mathbf{W}_\eta$ (avec $t_\theta = \tan\theta$) :

$$\begin{bmatrix} \dot{\phi} \\ \dot{\theta} \\ \dot{\psi} \end{bmatrix} = \mathbf{W}_\eta^{-1} \begin{bmatrix} p \\ q \\ r \end{bmatrix} = \begin{bmatrix} 1 & s_\phi t_\theta & c_\phi t_\theta \\ 0 & c_\phi & -s_\phi \\ 0 & \dfrac{s_\phi}{c_\theta} & \dfrac{c_\phi}{c_\theta} \end{bmatrix} \begin{bmatrix} p \\ q \\ r \end{bmatrix}$$

---

## MODULE 3 : Modélisation de la Propulsion et Méthode Universelle d'Allocation (Toute Géométrie : $+$, $\times$, Hexa, Octo)

### 1. Aérodynamique d'un groupe motopropulseur isolé

Un moteur électrique brushless $i$ entraînant une hélice fixe à la vitesse angulaire $\Omega_i$ (en $\text{rad/s}$) exerce sur le châssis du drone deux actions mécaniques issues de la théorie de l'élément de pale (*Blade Element Momentum Theory*) :

1. **La force de poussée aérodynamique $\mathbf{F}_i^B$ :**
L'hélice accélère une colonne d'air vers le bas ($+\mathbf{z}_B$). Par conservation de la quantité de mouvement, l'air exerce une force de portance vers le haut ($-\mathbf{z}_B$) proportionnelle au carré de la vitesse de rotation :

$$\mathbf{F}_i^B = \begin{bmatrix} 0 \\ 0 \\ -T_i \end{bmatrix} \quad \text{avec} \quad T_i = k_f \, \Omega_i^2$$



*(Où $k_f = C_T \rho A R^2$ en $\text{N}\cdot\text{s}^2/\text{rad}^2$ se détermine sur banc de traction avec une cellule de charge).*

2. **Le couple réactif de traînée d'hélice $\boldsymbol{\tau}_{\text{drag}, i}^B$ :**
Pour fendre l'air, le stator du moteur doit appliquer un couple électromagnétique à l'hélice. Par la **3ème loi de Newton (Action / Réaction)**, le rotor exerce sur le stator (donc sur le bras du drone) un couple exactement **opposé au sens de rotation de l'hélice** :

$$\tau_{\text{drag}, i} = \sigma_i \, k_m \, \Omega_i^2$$



où $k_m$ (en $\text{N}\cdot\text{m}\cdot\text{s}^2/\text{rad}^2$) est le coefficient de couple aérodynamique, et $\sigma_i \in \{-1, +1\}$ dépend du sens de rotation vu du dessus :
* **Si l'hélice $i$ tourne dans le sens Anti-Horaire (CCW, *Counter-Clockwise*) :** En regardant le drone de dessus, l'hélice tourne vers la gauche (autour de $-\mathbf{z}_B$). Sa réaction sur le châssis pousse donc le châssis vers la droite, soit autour de **$+\mathbf{z}_B$** (lacet positif) $\implies \sigma_i = +1$.
* **Si l'hélice $i$ tourne dans le sens Horaire (CW, *Clockwise*) :** Sa réaction pousse le châssis vers la gauche, soit autour de **$-\mathbf{z}_B$** (lacet négatif) $\implies \sigma_i = -1$.



### 2. Méthode universelle pour construire la Matrice d'Allocation $\mathbf{M}$ de N'IMPORTE QUEL drone

Tu veux être autonome pour modéliser demain un quadricoptère en « $+$ », en « $\times$ », un drone asymétrique (type *Deadcat* où les bras avant sont plus écartés pour ne pas filmer les hélices) ou un hexacoptère ? **Voici la formule générale qui marche à tous les coups.**

Soit un drone possédant $N$ moteurs. Dans le plan $(\mathbf{x}_B, \mathbf{y}_B)$ du châssis, le moteur $i$ est situé à la position géométrique :


$$\mathbf{r}_i^B = \begin{bmatrix} x_i \\ y_i \\ 0 \end{bmatrix} = \begin{bmatrix} L_i \cos\alpha_i \\ L_i \sin\alpha_i \\ 0 \end{bmatrix}$$


où $L_i$ est la longueur du bras $i$ et $\alpha_i$ est l'angle du bras mesuré depuis l'axe avant $+\mathbf{x}_B$ vers la droite $+\mathbf{y}_B$.

**Quelle formule physique applique-t-on ?** Le moment généré au centre de gravité $G$ par la poussée $\mathbf{F}_i^B$ du moteur $i$ est donné par le **produit vectoriel du bras de levier par la force** :


$$\boldsymbol{\tau}_{\text{bras}, i}^B = \mathbf{r}_i^B \times \mathbf{F}_i^B = \begin{bmatrix} x_i \\ y_i \\ 0 \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ -T_i \end{bmatrix} = \begin{bmatrix} y_i \cdot (-T_i) - 0 \cdot 0 \\ 0 \cdot 0 - x_i \cdot (-T_i) \\ x_i \cdot 0 - y_i \cdot 0 \end{bmatrix} = \begin{bmatrix} -y_i \, T_i \\ +x_i \, T_i \\ 0 \end{bmatrix}$$

> **Interprétation physique limpide de ce résultat :**
> * **Moment de Roulis $\tau_\phi$ (1ère ligne) $= -y_i \, T_i$ :** Un moteur situé à droite ($y_i > 0$) qui pousse vers le haut ($T_i > 0$) soulève l'aile droite, ce qui crée un roulis vers la gauche (**négatif**). D'où le signe **$-$** devant $y_i$ !
> * **Moment de Tangage $\tau_\theta$ (2ème ligne) $= +x_i \, T_i$ :** Un moteur situé à l'avant ($x_i > 0$) qui pousse vers le haut ($T_i > 0$) soulève le nez du drone, ce qui crée un tangage cabré (**positif**). D'où le signe **$+$** devant $x_i$ !
> 
> 

En sommant sur les $N$ moteurs et en ajoutant le couple de lacet $\tau_{\text{drag}, i}$ sur l'axe $z_B$, le torseur de commande général pour **n'importe quel drone** s'écrit :

$$\begin{bmatrix} U_1 \\ U_2 \\ U_3 \\ U_4 \end{bmatrix} = \begin{bmatrix} T_{\text{tot}} \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \sum_{i=1}^{N} \begin{bmatrix} k_f \\ -y_i \, k_f \\ +x_i \, k_f \\ \sigma_i \, k_m \end{bmatrix} \Omega_i^2$$

### 3. Application comparée : Configuration « $+$ » vs Configuration « $\times$ »

Appliquons cette formule universelle avec des bras de longueur $L$ :

#### Cas A : Quadricoptère en Configuration « $+$ »

* **Moteur 1 (Avant, $\alpha_1 = 0^\circ$, CCW)** : $x_1 = +L, \; y_1 = 0, \; \sigma_1 = +1$
* **Moteur 2 (Droite, $\alpha_2 = 90^\circ$, CW)** : $x_2 = 0, \; y_2 = +L, \; \sigma_2 = -1$
* **Moteur 3 (Arrière, $\alpha_3 = 180^\circ$, CCW)** : $x_3 = -L, \; y_3 = 0, \; \sigma_3 = +1$
* **Moteur 4 (Gauche, $\alpha_4 = -90^\circ$, CW)** : $x_4 = 0, \; y_4 = -L, \; \sigma_4 = -1$

En remplaçant directement $(x_i, y_i, \sigma_i)$ dans la formule universelle :


$$\begin{bmatrix} U_1 \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \underbrace{\begin{bmatrix} k_f & k_f & k_f & k_f \\ 0 & -L k_f & 0 & +L k_f \\ +L k_f & 0 & -L k_f & 0 \\ +k_m & -k_m & +k_m & -k_m \end{bmatrix}}_{\mathbf{M}_+} \begin{bmatrix} \Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2 \end{bmatrix}$$

#### Cas B : Quadricoptère en Configuration « $\times$ » (Standard PX4 / Betaflight)

Les bras sont à $45^\circ$ des axes $\mathbf{x}_B$ et $\mathbf{y}_B$. Comme $\cos(45^\circ) = \sin(45^\circ) = \frac{\sqrt{2}}{2}$, posons le bras de levier projeté $d = L \frac{\sqrt{2}}{2}$ :

* **Moteur 1 (Avant-Droit, $\alpha_1 = +45^\circ$, CCW)** : $x_1 = +d, \; y_1 = +d, \; \sigma_1 = +1$
* **Moteur 2 (Arrière-Droit, $\alpha_2 = +135^\circ$, CW)** : $x_2 = -d, \; y_2 = +d, \; \sigma_2 = -1$
* **Moteur 3 (Arrière-Gauche, $\alpha_3 = -135^\circ$, CCW)** : $x_3 = -d, \; y_3 = -d, \; \sigma_3 = +1$
* **Moteur 4 (Avant-Gauche, $\alpha_4 = -45^\circ$, CW)** : $x_4 = +d, \; y_4 = -d, \; \sigma_4 = -1$

En injectant dans notre formule universelle $[-y_i k_f, +x_i k_f]^T$, aucune hésitation possible :


$$\begin{bmatrix} U_1 \\ \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} = \underbrace{\begin{bmatrix} k_f & k_f & k_f & k_f \\ -d k_f & -d k_f & +d k_f & +d k_f \\ +d k_f & -d k_f & -d k_f & +d k_f \\ +k_m & -k_m & +k_m & -k_m \end{bmatrix}}_{\mathbf{M}_\times} \begin{bmatrix} \Omega_1^2 \\ \Omega_2^2 \\ \Omega_3^2 \\ \Omega_4^2 \end{bmatrix}$$


*(On vérifie : en config $\times$, pour cabrer $\tau_\theta > 0$, on accélère les deux moteurs avant 1 et 4 et on ralentit les deux moteurs arrière 2 et 3. Pour rouler à droite $\tau_\phi > 0$, on accélère les deux moteurs de gauche 3 et 4 et on ralentit ceux de droite 1 et 2).*

---

## MODULE 4 : Dynamique de Translation — Obtenir les Accélérations Linéaires ($\ddot{\mathbf{p}}^W$ et $\dot{\mathbf{v}}^B$)

### 1. Pourquoi existe-t-il DEUX façons d'écrire les accélérations linéaires, et quand utiliser laquelle ?

En robotique aérienne, tu rencontreras deux formulations des accélérations de translation :

1. **La formulation en repère Monde $W$ ($\ddot{\mathbf{p}}^W = [\ddot{x}_W, \ddot{y}_W, \ddot{z}_W]^T$) :**
* *Quand l'utiliser ?* Pour **la synthèse de lois de commande en position** (PID, LQR, Backstepping) et pour **le modèle de prédiction de position d'un EKF**, car l'état de position $[x_W, y_W, z_W]^T$ est directement la double intégrale de $\ddot{\mathbf{p}}^W$.


2. **La formulation en repère Corps $B$ ($\dot{\mathbf{v}}^B = [\dot{u}, \dot{v}, \dot{w}]^T$) :**
* *Quand l'utiliser ?* Pour **modéliser finement l'aérodynamique** (la traînée de l'air dépend de la vitesse relative du vent sur chaque face du drone dans $B$) et pour **relier le modèle à l'accéléromètre embarqué** de l'IMU.



Démontrons les deux pas à pas.

### 2. Démonstration de la Formulation 1 : Accélérations dans le repère Monde $W$

* **Loi physique appliquée :** La 2ème loi de Newton (Principe Fondamental de la Dynamique en translation), qui stipule que **dans un repère galiléen fixe ($W$)**, la masse multipliée par l'accélération absolue est égale à la somme des forces extérieures exprimées dans $W$ :

$$m \, \ddot{\mathbf{p}}^W = \mathbf{F}_{\text{gravité}}^W + \mathbf{F}_{\text{poussée}}^W + \mathbf{F}_{\text{traînée}}^W$$


* **Traduction de chaque force dans le repère $W$ :**
1. **Gravité :** Déjà dans $W$ (pointe vers le bas $+\mathbf{z}_W$) :

$$\mathbf{F}_{\text{gravité}}^W = \begin{bmatrix} 0 \\ 0 \\ +m g \end{bmatrix}$$


2. **Poussée des rotors :** Connue dans $B$ ($\mathbf{F}_{\text{poussée}}^B = [0, 0, -U_1]^T$). On la transporte dans $W$ en la multipliant par $\mathbf{R}_B^W$ :

$$\mathbf{F}_{\text{poussée}}^W = \mathbf{R}_B^W \begin{bmatrix} 0 \\ 0 \\ -U_1 \end{bmatrix} = -U_1 \times \left(\text{3ème colonne de }\mathbf{R}_B^W\right) = -U_1 \begin{bmatrix} c_\psi s_\theta c_\phi + s_\psi s_\phi \\ s_\psi s_\theta c_\phi - c_\psi s_\phi \\ c_\theta c_\phi \end{bmatrix}$$


3. **Traînée aérodynamique linéaire :** Force de frottement visqueux opposée au déplacement :

$$\mathbf{F}_{\text{traînée}}^W = -\mathbf{K}_d \, \dot{\mathbf{p}}^W = -\begin{bmatrix} k_{dx} \dot{x}_W \\ k_{dy} \dot{y}_W \\ k_{dz} \dot{z}_W \end{bmatrix}$$




* **Équation finale en divisant par la masse $m$ :**

$$\begin{cases}
\ddot{x}_W = -\dfrac{U_1}{m} \left(\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi\right) - \dfrac{k_{dx}}{m} \dot{x}_W \\[8pt]
\ddot{y}_W = -\dfrac{U_1}{m} \left(\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi\right) - \dfrac{k_{dy}}{m} \dot{y}_W \\[8pt]
\ddot{z}_W = g - \dfrac{U_1}{m} \left(\cos\theta \cos\phi\right) - \dfrac{k_{dz}}{m} \dot{z}_W
\end{cases}$$



### 3. Démonstration de la Formulation 2 : Accélérations dans le repère Corps $B$ (et apparition de Coriolis)

Notons $\mathbf{v}^B = [u, v, w]^T$ le vecteur vitesse linéaire du centre de masse projeté sur les axes $(\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B)$ du drone.

* **Le piège mathématique :** Newton dit $\sum \mathbf{F} = m \left(\frac{d\mathbf{v}}{dt}\right)_W$. Or, si l'on dérive les composantes $[u, v, w]^T$ à l'intérieur du repère $B$, on calcule $\left(\frac{d\mathbf{v}}{dt}\right)_B = [\dot{u}, \dot{v}, \dot{w}]^T$. Comme le repère $B$ tourne lui-même à la vitesse angulaire $\boldsymbol{\omega}^B = [p, q, r]^T$, la dérivée vue depuis $B$ oublie la rotation des axes !
* **Quelle formule applique-t-on pour corriger cela ? La Formule de Dérivation Vectorielle de Bour :**
Pour **tout vecteur quelconque $\mathbf{A}$**, la relation entre sa dérivée temporelle dans le repère fixe $W$ et sa dérivée temporelle dans le repère tournant $B$ est :

$$\left(\frac{d\mathbf{A}}{dt}\right)_W = \left(\frac{d\mathbf{A}}{dt}\right)_B + \boldsymbol{\omega}_{B/W} \times \mathbf{A}$$


* **Application de la formule de Bour au vecteur vitesse $\mathbf{v}^B$ :**

$$\left(\frac{d\mathbf{v}}{dt}\right)_W^B = \dot{\mathbf{v}}^B + \boldsymbol{\omega}^B \times \mathbf{v}^B$$



La 2ème loi de Newton projetée dans $B$ s'écrit donc :

$$m \left(\dot{\mathbf{v}}^B + \boldsymbol{\omega}^B \times \mathbf{v}^B\right) = \mathbf{F}_{\text{gravité}}^B + \mathbf{F}_{\text{poussée}}^B + \mathbf{F}_{\text{traînée}}^B$$


* **Traduction de chaque terme dans $B$ :**
1. **Terme d'entraînement centripète/Coriolis ($\boldsymbol{\omega}^B \times \mathbf{v}^B$) :**

$$\begin{bmatrix} p \\ q \\ r \end{bmatrix} \times \begin{bmatrix} u \\ v \\ w \end{bmatrix} = \begin{bmatrix} q w - r v \\ r u - p w \\ p v - q u \end{bmatrix}$$


2. **Gravité projetée dans $B$ :** On utilise la matrice inverse $\mathbf{R}_W^B = \left(\mathbf{R}_B^W\right)^T$ appliquée à $\mathbf{g}^W = [0, 0, +g]^T$ (ce qui sélectionne la 3ème colonne de $\mathbf{R}_W^B$, c'est-à-dire la **3ème ligne de $\mathbf{R}_B^W$** !) :

$$\mathbf{F}_{\text{gravité}}^B = \mathbf{R}_W^B \begin{bmatrix} 0 \\ 0 \\ m g \end{bmatrix} = m g \begin{bmatrix} -\sin\theta \\ \cos\theta \sin\phi \\ \cos\theta \cos\phi \end{bmatrix}$$


3. **Poussée dans $B$ :** Déjà dans $B$ ! Pas de matrice de rotation :

$$\mathbf{F}_{\text{poussée}}^B = \begin{bmatrix} 0 \\ 0 \\ -U_1 \end{bmatrix}$$




* **Équation finale dans $B$ (en isolant $\dot{u}, \dot{v}, \dot{w}$) :**

$$\begin{cases}
\dot{u} = r v - q w - g \sin\theta - \dfrac{k_{dx}}{m} u \\[8pt]
\dot{v} = p w - r u + g \cos\theta \sin\phi - \dfrac{k_{dy}}{m} v \\[8pt]
\dot{w} = q u - p v + g \cos\theta \cos\phi - \dfrac{U_1}{m} - \dfrac{k_{dz}}{m} w
\end{cases}$$



---

## MODULE 5 : Dynamique de Rotation — Le Bilan Complet des Moments ($\dot{\boldsymbol{\omega}}^B$)

### 1. Pourquoi écrit-on TOUJOURS l'équation des moments dans le repère $B$ ?

Le moment cinétique d'un corps rigide autour de son centre de masse $G$ est $\mathbf{L} = \mathbf{I} \, \boldsymbol{\omega}$, où $\mathbf{I}$ est le tenseur (matrice $3 \times 3$) d'inertie :


$$\mathbf{I}_B = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \\ -I_{yx} & I_{yy} & -I_{yz} \\ -I_{zx} & -I_{zy} & I_{zz} \end{bmatrix}$$

* Dans le repère terrestre $W$, comme le drone tourne, ses bras changent d'orientation par rapport au Nord et à l'Est : $\mathbf{I}_W(t) = \mathbf{R}_B^W(t) \, \mathbf{I}_B \, \mathbf{R}_W^B(t)$ varie à chaque instant.
* Dans le repère Corps $B$, les moteurs et la batterie ne bougent pas par rapport aux axes $(\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B)$ : **$\mathbf{I}_B$ est constante**. De plus, si les plans $(x_B, z_B)$ et $(y_B, z_B)$ sont des plans de symétrie mécanique du drone, tous les produits d'inertie hors diagonale s'annulent et $\mathbf{I}_B$ devient **diagonale** :

$$\mathbf{I}_B = \begin{bmatrix} I_{xx} & 0 & 0 \\ 0 & I_{yy} & 0 \\ 0 & 0 & I_{zz} \end{bmatrix}$$



### 2. Quelle loi physique applique-t-on ?

On applique le **Théorème du Moment Cinétique (Loi d'Euler)** au centre de gravité $G$ par rapport au repère galiléen $W$ :


$$\sum \boldsymbol{\tau}_{\text{ext}}^B = \left(\frac{d\mathbf{L}_{\text{tot}}}{dt}\right)_W^B$$

Or, un drone est composé de **deux sous-ensembles en rotation** :

1. **Le châssis rigide**, de moment cinétique $\mathbf{L}_{\text{châssis}}^B = \mathbf{I}_B \, \boldsymbol{\omega}^B$.
2. **Les $N$ rotors (hélices + cloches moteurs)**, chacun ayant un moment d'inertie axial $J_r$ autour de son axe vertical $\mathbf{z}_B$ et tournant à la vitesse propre $\pm \Omega_i$. Leur moment cinétique combiné le long de $\mathbf{z}_B$ vaut :

$$\mathbf{L}_{\text{rotors}}^B = \begin{bmatrix} 0 \\ 0 \\ J_r \, \Omega_r \end{bmatrix} \quad \text{avec} \quad \Omega_r = \sum_{i=1}^{N} (-\sigma_i) \, \Omega_i$$



*(Où $-\sigma_i = +1$ pour un rotor CW qui tourne dans le sens $+\mathbf{z}_B$, et $-1$ pour un rotor CCW).*

### 3. Application de la Formule de Bour au Moment Cinétique Total

Appliquons la formule de dérivation dans un repère mobile $\left(\frac{d\mathbf{L}}{dt}\right)_W^B = \left(\frac{d\mathbf{L}}{dt}\right)_B + \boldsymbol{\omega}^B \times \mathbf{L}^B$ aux deux sous-ensembles :

1. **Dérivée du moment cinétique du châssis :**

$$\left(\frac{d\mathbf{L}_{\text{châssis}}}{dt}\right)_W^B = \mathbf{I}_B \, \dot{\boldsymbol{\omega}}^B + \boldsymbol{\omega}^B \times \left(\mathbf{I}_B \, \boldsymbol{\omega}^B\right)$$



Calculons explicitement le produit vectoriel du **couplage inertiel centrifuge** :

$$\boldsymbol{\omega}^B \times \left(\mathbf{I}_B \, \boldsymbol{\omega}^B\right) = \begin{bmatrix} p \\ q \\ r \end{bmatrix} \times \begin{bmatrix} I_{xx} p \\ I_{yy} q \\ I_{zz} r \end{bmatrix} = \begin{bmatrix} q (I_{zz} r) - r (I_{yy} q) \\ r (I_{xx} p) - p (I_{zz} r) \\ p (I_{yy} q) - q (I_{xx} p) \end{bmatrix} = \begin{bmatrix} (I_{zz} - I_{yy}) \, q \, r \\ (I_{xx} - I_{zz}) \, p \, r \\ (I_{yy} - I_{xx}) \, p \, q \end{bmatrix}$$


2. **Dérivée du moment cinétique des rotors (Effet Gyroscopique) :**
En négligeant le terme d'accélération interne des rotors devant la précession gyroscopique :

$$\boldsymbol{\omega}^B \times \mathbf{L}_{\text{rotors}}^B = \begin{bmatrix} p \\ q \\ r \end{bmatrix} \times \begin{bmatrix} 0 \\ 0 \\ J_r \, \Omega_r \end{bmatrix} = \begin{bmatrix} q \, J_r \, \Omega_r \\ -p \, J_r \, \Omega_r \\ 0 \end{bmatrix}$$



*(C'est l'effet gyroscopique : si les rotors ont une vitesse résiduelle $\Omega_r \neq 0$ et que le drone tangue à la vitesse $q$, cela crée un couple parasite sur l'axe de roulis $p$ !)*

### 4. Assemblage et isolement des accélérations angulaires $[\dot{p}, \dot{q}, \dot{r}]^T$

L'équation d'équilibre d'Euler s'écrit :


$$\mathbf{I}_B \, \dot{\boldsymbol{\omega}}^B + \begin{bmatrix} (I_{zz} - I_{yy}) \, q \, r \\ (I_{xx} - I_{zz}) \, p \, r \\ (I_{yy} - I_{xx}) \, p \, q \end{bmatrix} + \begin{bmatrix} q \, J_r \, \Omega_r \\ -p \, J_r \, \Omega_r \\ 0 \end{bmatrix} = \begin{bmatrix} \tau_\phi \\ \tau_\theta \\ \tau_\psi \end{bmatrix} - \begin{bmatrix} k_{rp} \, p \\ k_{rq} \, q \\ k_{rr} \, r \end{bmatrix}$$


*(où $[k_{rp}p, k_{rq}q, k_{rr}r]^T$ est l'amortissement aérodynamique en rotation).*

En passant les termes inertiels et gyroscopiques à droite du signe « $=$ » et en divisant par $I_{xx}, I_{yy}, I_{zz}$, on obtient **le système final des accélérations angulaires** :

$$\begin{cases}
\dot{p} = \dfrac{I_{yy} - I_{zz}}{I_{xx}} \, q \, r - \dfrac{J_r}{I_{xx}} \, q \, \Omega_r + \dfrac{\tau_\phi}{I_{xx}} - \dfrac{k_{rp}}{I_{xx}} \, p \\[8pt]
\dot{q} = \dfrac{I_{zz} - I_{xx}}{I_{yy}} \, p \, r + \dfrac{J_r}{I_{yy}} \, p \, \Omega_r + \dfrac{\tau_\theta}{I_{yy}} - \dfrac{k_{rq}}{I_{yy}} \, q \\[8pt]
\dot{r} = \dfrac{I_{xx} - I_{yy}}{I_{zz}} \, p \, q + \dfrac{\tau_\psi}{I_{zz}} - \dfrac{k_{rr}}{I_{zz}} \, r
\end{cases}$$

---

## MODULE 6 : Mise en Équation d'État ($\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$) et Passage au Filtre de Kalman Étendu (EKF)

C'est ici que tes compétences d'ingénieur robotique prennent tout leur sens. Comment transforme-t-on ces équations physiques en un **Filtre de Kalman Étendu (EKF)** fonctionnel ?

### 1. Le Secret d'Architecture d'un EKF de Drone : "IMU-Driven Prediction"

Dans un cours théorique d'automatique, l'entrée du système est la vitesse des moteurs $\mathbf{u} = [\Omega_1, \Omega_2, \Omega_3, \Omega_4]^T$.
Mais **en robotique réelle**, utiliser les commandes moteurs dans l'étape de prédiction d'un EKF fonctionne mal car les coefficients aérodynamiques $k_f, k_m$, les courants de vent et la tension batterie sont incertains.

**Comment fait-on dans l'industrie (PX4 ECL EKF2 / ArduPilot) ?**
On remplace le modèle des moteurs par les mesures haute fréquence ($200\text{ à }1000\text{ Hz}$) de la centrale inertielle (IMU) !

* **Ce que mesure réellement un accéléromètre ($\mathbf{a}_{\text{imu}}^B$) :** Un accéléromètre MEMS est une masse-ressort. En chute libre ($\ddot{\mathbf{p}}^W = \mathbf{g}^W$), le ressort est détendu et l'accéléromètre affiche $\mathbf{0}$ ! Il ne mesure donc pas l'accélération absolue, mais la **force spécifique** (toutes les forces *sauf* la gravité, divisées par la masse, exprimées dans $B$) :

$$\mathbf{a}_{\text{imu}}^B = \mathbf{R}_W^B \left(\ddot{\mathbf{p}}^W - \mathbf{g}^W\right) = \frac{\mathbf{F}_{\text{poussée}}^B + \mathbf{F}_{\text{traînée}}^B}{m}$$


* **Conséquence magique :** En isolant $\ddot{\mathbf{p}}^W$ dans cette formule, on obtient l'accélération terrestre **exacte** sans avoir besoin de connaître ni $U_1$, ni $k_f$, ni la traînée :

$$\ddot{\mathbf{p}}^W = \mathbf{R}_B^W \, \mathbf{a}_{\text{imu}}^B + \mathbf{g}^W$$



### 2. Les Deux Formulations d'État selon ton besoin

#### Option 1 : Modèle Dynamique Complet (pour Simulateur Physique, LQR, MPC ou Observateur d'Actionneurs)

* **Vecteur d'état ($12 \times 1$) :** $\mathbf{x} = \left[x_W, y_W, z_W, \; \dot{x}_W, \dot{y}_W, \dot{z}_W, \; \phi, \theta, \psi, \; p, q, r\right]^T$
* **Vecteur d'entrée ($4 \times 1$) :** $\mathbf{u} = \left[U_1, \tau_\phi, \tau_\theta, \tau_\psi\right]^T$
* **Fonction d'évolution continue $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$ :**

$$\dot{\mathbf{x}} = \begin{bmatrix} \dot{\mathbf{p}}^W \\ \mathbf{g}^W + \frac{1}{m} \mathbf{R}_B^W(\boldsymbol{\eta}) \begin{bmatrix}0\\0\\-U_1\end{bmatrix} - \frac{1}{m}\mathbf{K}_d \dot{\mathbf{p}}^W \\ \mathbf{W}_\eta^{-1}(\boldsymbol{\eta}) \, \boldsymbol{\omega}^B \\ \mathbf{I}_B^{-1} \left( \boldsymbol{\tau}^B - \boldsymbol{\omega}^B \times (\mathbf{I}_B \boldsymbol{\omega}^B) - \boldsymbol{\tau}_{\text{gyro}}^B \right) \end{bmatrix}$$



#### Option 2 : Modèle Cinématique Piloté par IMU (Standard EKF de Navigation)

* **Vecteur d'état réduit ($9 \times 1$, ou $15 \times 1$ si on ajoute les biais capteurs $\mathbf{b}_a, \mathbf{b}_g$) :**

$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^W \\ \mathbf{v}^W \\ \boldsymbol{\eta} \end{bmatrix} = \left[x_W, y_W, z_W, \; \dot{x}_W, \dot{y}_W, \dot{z}_W, \; \phi, \theta, \psi\right]^T$$


* **Vecteur d'entrée mesuré par l'IMU ($6 \times 1$) :**

$$\mathbf{u}_{\text{imu}} = \begin{bmatrix} \mathbf{a}_{\text{imu}}^B \\ \boldsymbol{\omega}_{\text{imu}}^B \end{bmatrix} = \left[a_{xB}, a_{yB}, a_{zB}, \; p, q, r\right]^T$$


* **Équation d'état continue $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}_{\text{imu}})$ :**

$$\dot{\mathbf{x}} = \begin{bmatrix} \mathbf{v}^W \\ \mathbf{R}_B^W(\phi, \theta, \psi) \, \mathbf{a}_{\text{imu}}^B + \begin{bmatrix} 0 \\ 0 \\ +g \end{bmatrix} \\ \mathbf{W}_\eta^{-1}(\phi, \theta) \, \boldsymbol{\omega}_{\text{imu}}^B \end{bmatrix}$$



### 3. Les 3 Étapes pour coder ton EKF à partir de $\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u})$

#### Étape A : Discrétisation temporelle (Pas d'échantillonnage $\Delta t$)

Un microcontrôleur tourne en temps discret à la période $\Delta t$ (ex: $\Delta t = 0.005\text{ s}$ à $200\text{ Hz}$). Par intégration d'Euler au premier ordre :


$$\mathbf{x}_{k\vert{}k-1} = f_d(\mathbf{x}_{k-1}, \mathbf{u}_k) = \mathbf{x}_{k-1} + f(\mathbf{x}_{k-1}, \mathbf{u}_k) \cdot \Delta t$$

#### Étape B : Calcul de la Matrice Jacobienne d'État $\mathbf{F}_k$

Pour propager la matrice de covariance de l'erreur $\mathbf{P}_{k\vert{}k-1} = \mathbf{F}_k \mathbf{P}_{k-1} \mathbf{F}_k^T + \mathbf{Q}_k$, il faut linéariser le système autour de l'état courant en calculant la matrice des dérivées partielles :


$$\mathbf{A}(\mathbf{x}, \mathbf{u}) = \frac{\partial f(\mathbf{x}, \mathbf{u})}{\partial \mathbf{x}} \quad \implies \quad \mathbf{F}_k = \mathbf{I}_{n \times n} + \mathbf{A}(\mathbf{x}_{k-1}, \mathbf{u}_k) \cdot \Delta t$$

Pour l'Option 2 ($9$ états : $\mathbf{p}^W, \mathbf{v}^W, \boldsymbol{\eta}$), la structure par blocs $3 \times 3$ de la Jacobienne continue $\mathbf{A}$ est remarquablement propre :


$$\mathbf{A} = \frac{\partial f}{\partial \mathbf{x}} = \begin{bmatrix} \dfrac{\partial \dot{\mathbf{p}}^W}{\partial \mathbf{p}^W} & \dfrac{\partial \dot{\mathbf{p}}^W}{\partial \mathbf{v}^W} & \dfrac{\partial \dot{\mathbf{p}}^W}{\partial \boldsymbol{\eta}} \\ \dfrac{\partial \dot{\mathbf{v}}^W}{\partial \mathbf{p}^W} & \dfrac{\partial \dot{\mathbf{v}}^W}{\partial \mathbf{v}^W} & \dfrac{\partial \dot{\mathbf{v}}^W}{\partial \boldsymbol{\eta}} \\ \dfrac{\partial \dot{\boldsymbol{\eta}}}{\partial \mathbf{p}^W} & \dfrac{\partial \dot{\boldsymbol{\eta}}}{\partial \mathbf{v}^W} & \dfrac{\partial \dot{\boldsymbol{\eta}}}{\partial \boldsymbol{\eta}} \end{bmatrix} = \begin{bmatrix} \mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} \\ \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \dfrac{\partial \left(\mathbf{R}_B^W \mathbf{a}_{\text{imu}}^B\right)}{\partial \boldsymbol{\eta}} \\ \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \dfrac{\partial \left(\mathbf{W}_\eta^{-1} \boldsymbol{\omega}_{\text{imu}}^B\right)}{\partial \boldsymbol{\eta}} \end{bmatrix}$$

> **Comment obtient-on le bloc $\frac{\partial (\mathbf{R}_B^W \mathbf{a}^B)}{\partial \boldsymbol{\eta}}$ (ou avec l'entrée $-U_1$ autour du vol stationnaire) ?**
> Autour d'un vol quasi-stationnaire ($\phi \approx 0, \theta \approx 0$ et $a_{zB} \approx -U_1/m \approx -g$), en dérivant la 3ème colonne de $\mathbf{R}_B^W$ par rapport à $\phi, \theta, \psi$, on obtient directement le couplage entre l'inclinaison et l'accélération horizontale :
> 
> $$\left.\frac{\partial \dot{\mathbf{v}}^W}{\partial \boldsymbol{\eta}}\right\vert{}_{\text{hover}} = \begin{bmatrix} -g \sin\psi & -g \cos\psi & 0 \\ g \cos\psi & -g \sin\psi & 0 \\ 0 & 0 & 0 \end{bmatrix}$$
> 
> 
> 
> *C'est grâce à ce bloc non nul dans $\mathbf{F}_k$ qu'une mesure de position/vitesse GPS dans l'EKF est capable de corriger la dérive des angles de roulis $\phi$ et tangage $\theta$ !*

#### Étape C : Modèle d'Observation $\mathbf{z}_k = h(\mathbf{x}_k) + \mathbf{v}_k$ et Jacobienne $\mathbf{H}_k = \frac{\partial h}{\partial \mathbf{x}}$

* **Si le capteur est un GPS + Baromètre (mesure Position et Vitesse dans $W$) :**

$$\mathbf{z}_{\text{gps}} = \begin{bmatrix} \mathbf{p}^W \\ \mathbf{v}^W \end{bmatrix} \quad \implies \quad \mathbf{H}_{\text{gps}} = \begin{bmatrix} \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} \\ \mathbf{0}_{3\times 3} & \mathbf{I}_{3\times 3} & \mathbf{0}_{3\times 3} \end{bmatrix}$$


* **Si le capteur est un Magnétomètre 3D (mesure le champ magnétique terrestre $\mathbf{m}^W$ projeté dans $B$) :**

$$h_{\text{mag}}(\mathbf{x}) = \mathbf{R}_W^B(\phi, \theta, \psi) \, \mathbf{m}^W \quad \implies \quad \mathbf{H}_{\text{mag}} = \begin{bmatrix} \mathbf{0}_{3\times 3} & \mathbf{0}_{3\times 3} & \dfrac{\partial \left(\mathbf{R}_W^B \mathbf{m}^W\right)}{\partial \boldsymbol{\eta}} \end{bmatrix}$$



---

## Laboratoire Interactif : Mixage Multi-Configurations ($+$ vs $\times$) & Jacobienne EKF

Utilisez l'outil interactif ci-dessous pour tester concrètement le passage entre une géométrie **Quad $+$**, **Quad $\times$** ou **Hexacoptère**, et inspecter en temps réel l'impact d'une variation des vitesses moteurs sur la matrice d'allocation $\mathbf{M}$, les accélérations en repères $B$ et $W$, ainsi que les blocs de la matrice Jacobienne $\mathbf{A}$ de l'EKF :

---

## Aide-Mémoire Synthétique (Cheat-Sheet d'Ingénieur)

| Étape de Modélisation | Repère de travail | Formule / Loi Fondamentale appliquée | Équation Maîtresse obtenue |
| --- | --- | --- | --- |
| **1. Orientation** | $B \to W$ | Séquence intrinsèque Z-Y-X | $\mathbf{R}_B^W = \mathbf{R}_z(\psi)\mathbf{R}_y(\theta)\mathbf{R}_x(\phi)$ |
| **2. Cinématique Attitude** | $B \to \text{Euler}$ | Projection des rotations intermédiaires dans $B$ | $\dot{\boldsymbol{\eta}} = \mathbf{W}_\eta^{-1}(\phi,\theta) \, \boldsymbol{\omega}^B$ |
| **3. Allocation Moteurs** | $B$ (FRD) | Produit vectoriel $\boldsymbol{\tau}_i^B = \mathbf{r}_i^B \times \mathbf{F}_i^B$ | $\tau_\phi = \sum -y_i T_i \quad ; \quad \tau_\theta = \sum +x_i T_i$ |
| **4a. Translation Monde** | $W$ (NED) | 2ème Loi de Newton dans un repère fixe | $m\ddot{\mathbf{p}}^W = \begin{bmatrix}0\\0\\mg\end{bmatrix} + \mathbf{R}_B^W \begin{bmatrix}0\\0\\-U_1\end{bmatrix} - \mathbf{K}_d \dot{\mathbf{p}}^W$ |
| **4b. Translation Corps** | $B$ (FRD) | Newton + Théorème de Bour ($\boldsymbol{\omega}^B \times \mathbf{v}^B$) | $m(\dot{\mathbf{v}}^B + \boldsymbol{\omega}^B \times \mathbf{v}^B) = \mathbf{R}_W^B \begin{bmatrix}0\\0\\mg\end{bmatrix} + \begin{bmatrix}0\\0\\-U_1\end{bmatrix} - \mathbf{K}_d \mathbf{v}^B$ |
| **5. Rotation Corps** | $B$ (FRD) | Loi d'Euler + Théorème de Bour ($\boldsymbol{\omega}^B \times \mathbf{I}_B\boldsymbol{\omega}^B$) | $\mathbf{I}_B \dot{\boldsymbol{\omega}}^B = \boldsymbol{\tau}^B - \boldsymbol{\omega}^B \times (\mathbf{I}_B \boldsymbol{\omega}^B) - \boldsymbol{\omega}^B \times \mathbf{L}_{\text{rotors}}^B$ |
| **6. Prédiction EKF** | $B \to W$ | Modèle de mesure de force spécifique IMU | $\dot{\mathbf{v}}^W = \mathbf{R}_B^W \left(\mathbf{a}_{\text{imu}}^B - \mathbf{b}_a\right) + \mathbf{g}^W$ |


Voici les deux modules entièrement formatés en syntaxe Markdown pure (sans balises d'interface visuelle). Tu n'as plus qu'à copier le bloc de code ci-dessous et le coller directement à la fin de ton fichier `.md` !

# MODULE 7 : Chronologie Logique et Causale — L'Algorithme Pas-à-Pas (Le "Pourquoi du Comment")

**Philosophie :** En mathématiques, on peut écrire des équations dans n'importe quel ordre. En informatique embarquée (dans le code d'un Pixhawk ou d'un Filtre de Kalman), **l'ordre de calcul est strictement imposé par la causalité physique**. On ne peut pas calculer une force dans le repère World sans avoir la Matrice de Rotation, on ne peut pas avoir la Matrice sans les Angles, et on ne peut pas avoir les Angles sans intégrer le Gyroscope. 

Voici la boucle de calcul (qui tourne à chaque milliseconde $\Delta t$) depuis la lecture des capteurs jusqu'à la prédiction de la position future.

### 1. Lecture des capteurs et de la commande (Acquisition)
*Tout commence dans le repère Body*
* **Objectif :** Connaître l'état instantané des actionneurs et de la rotation.
* **Action :** Le drone lit les courants envoyés aux moteurs pour calculer la poussée totale ($T_{tot}$) et les 3 moments ($\tau_\phi, \tau_\theta, \tau_\psi$) exprimés dans $B$. Le gyroscope lit la vitesse angulaire instantanée $\omega_B = [p, q, r]^T$.

### 2. Calcul des accélérations angulaires (Le TMD)
*Pourquoi on utilise la formule de Bour*
* **Objectif :** Savoir comment la vitesse de rotation va changer ($\dot{p}, \dot{q}, \dot{r}$).
* **Le Problème :** La loi fondamentale ($\sum M = dL/dt$) s'applique dans le repère absolu $W$. Mais dans $W$, la matrice d'inertie $I$ du drone change tout le temps.
* **La Solution (Bour) :** On reste dans le repère $B$ où $I_B$ est une constante. On applique la loi d'Euler modifiée par Bour : $\sum M_B = I_B \dot{\omega}_B + \omega_B \times (I_B \omega_B)$.
* **Le Résultat :** On isole $\dot{\omega}_B$. En la multipliant par le temps $\Delta t$, on connaît la vitesse angulaire du gyroscope pour la milliseconde suivante.

### 3. Mise à jour des Angles d'Euler (La Cinématique)
*Le paradoxe de l'œuf et de la poule*
* **Objectif :** On doit construire la Matrice de Rotation $R_B^W$ pour plus tard, il nous faut donc nos angles d'Euler ($\phi, \theta, \psi$).
* **Le Problème :** Le gyroscope ne crache que des vitesses locales ($p, q, r$). Il faut les convertir en vitesses d'Euler ($\dot{\phi}, \dot{\theta}, \dot{\psi}$) via la matrice $W_\eta^{-1}$.
* **Le Piège :** La matrice $W_\eta^{-1}$ a besoin des angles d'Euler pour fonctionner (elle est pleine de $\cos$ et $\sin$) ! 
* **La Solution (Intégration) :** On utilise les angles d'Euler *calculés à l'étape précédente* pour remplir la matrice. On multiplie par $[p, q, r]^T$, on obtient $[\dot{\phi}, \dot{\theta}, \dot{\psi}]^T$, on multiplie par $\Delta t$, et on additionne pour trouver les **nouveaux** angles d'Euler de l'instant présent.

### 4. Construction de la Matrice de Rotation
*L'outil de projection*
* **Objectif :** Créer la "passerelle" mathématique entre le drone et la Terre.
* **Action :** Maintenant que l'on possède nos nouveaux angles d'Euler purs $(\phi, \theta, \psi)$, on les injecte dans les sinus et cosinus de la grande matrice de rotation $R_B^W$. 
* **Le Résultat :** La matrice est prête à l'emploi. L'orientation du drone est officiellement résolue.

### 5. Calcul des accélérations linéaires (Le PFD)
*Le pont entre Body et World*
* **Objectif :** Savoir de combien de centimètres le drone va avancer, reculer ou tomber ($\ddot{x}, \ddot{y}, \ddot{z}$).
* **Le Problème :** La poussée des moteurs "vit" dans le repère $B$ (vers le ventre du drone), mais la gravité "vit" dans le repère $W$ (vers le centre de la Terre). On ne peut pas les additionner directement.
* **La Solution :** On prend la poussée des moteurs ($B$), on la multiplie par notre Matrice $R_B^W$ fraîchement calculée pour la "tordre" dans le repère $W$. On lui additionne la gravité ($W$), et on divise par la masse.
* **Le Résultat final :** On obtient l'accélération linéaire absolue. En intégrant deux fois (par $\Delta t$), on obtient la nouvelle vitesse et la nouvelle position GPS du drone. La prédiction est terminée !

***

# MODULE 8 : Le "Bêtisier" des Confusions (Les Pièges à Éviter)

Cette section recense les intuitions trompeuses classiques lorsque l'on passe de la mécanique du point (2D) à la mécanique du solide rigide (3D). 

### 1. Confusion entre "Moment" et "Moment Cinétique"
* **L'Erreur :** Penser que le moment cinétique est la force qui fait tourner le drone.
* **La Réalité Physique :** 
  * Le **Moment** (Couple / Torque / $\tau$) est la cause : c'est l'effort appliqué (Force $\times$ Bras de levier).
  * Le **Moment Cinétique** ($L = I\omega$) est la conséquence : c'est "l'élan de rotation" ou "l'inertie en mouvement" accumulé par l'objet. 
  * La loi d'Euler relie les deux : La somme des Moments appliqués modifie le Moment Cinétique au cours du temps ($\sum M = dL/dt$).

### 2. Oublier l'effet centrifuge en 3D (Le syndrome $F = ma$)
* **L'Erreur :** Écrire la loi de rotation du drone sous la forme simple $\sum M = I \cdot \dot{\omega}$ (comme on écrirait $F = ma$).
* **La Réalité Physique :** Cette formule n'est vraie qu'en 2D, ou pour une sphère parfaite, ou si le drone ne tourne que sur un seul axe à la fois. En 3D, dès que le drone tourne sur deux axes simultanément, les masses en rotation sont déviées, créant une précession (effet gyroscopique/Coriolis). Il faut **obligatoirement** ajouter le terme de couplage inertiel $\vec{\omega} \times (I\vec{\omega})$.

### 3. Chercher à calculer $L_{World}$ pour appliquer la formule de Bour
* **L'Erreur :** La loi d'Euler exige la dérivée de $L_{World}$. Croire qu'il faut calculer la valeur de $L_{World}$ (avec $I_{World}$) pour pouvoir ensuite appliquer la formule de Bour.
* **La Réalité Physique :** C'est l'inverse ! Le but de la formule de Bour est de nous **dispenser totalement** de calculer quoi que ce soit dans World. Bour garantit que si on fait tout notre calcul dans Body (dérivée de $L_{Body} + \omega_B \times L_{Body}$), le chiffre obtenu sera magiquement égal à ce qu'aurait trouvé un observateur cherchant à calculer la dérivée dans World. On ne cherche donc jamais à connaître $L_{World}$.

### 4. Croire que les angles d'Euler sont des "mesures directes"
* **L'Erreur :** Visualiser les angles $\phi, \theta, \psi$ comme trois angles mesurés simultanément entre le châssis et le sol (comme un rapporteur 2D).
* **La Réalité Physique :** En 3D, les rotations ne commutent pas. Les angles d'Euler sont une **séquence de charnières** (Z puis Y puis X). Le Tangage n'est pas mesuré par rapport au Monde, mais par rapport à un repère intermédiaire qui a *déjà* subi le Lacet. C'est pour cela que la vitesse du gyroscope (instantanée) ne correspond pas à la dérivée des angles d'Euler (séquentielle).

### 5. Confondre "Cinématique d'orientation" et "Formule de Bour"
* **L'Erreur :** Croire que l'on utilise la Formule de Bour (Coriolis) pour trouver la matrice de transfert $W_\eta$ entre le gyroscope et les angles d'Euler.
* **La Réalité Physique :** La formule de Bour s'applique uniquement quand on **dérive** un vecteur d'un repère tournant à un repère fixe (Dynamique / Accélération). Pour passer des vitesses gyroscopiques aux vitesses d'Euler (Cinématique), on utilise une simple loi géométrique d'addition de vecteurs vitesses, que l'on projette dans le repère Body grâce à des matrices de rotation partielles.

```
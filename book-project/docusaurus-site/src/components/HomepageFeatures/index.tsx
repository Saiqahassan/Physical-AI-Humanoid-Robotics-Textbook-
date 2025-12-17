import React from 'react';

import styles from './styles.module.css';

export default function HomepageFeatures(): React.ReactElement {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">Feature 1</div>
          <div className="col col--4">Feature 2</div>
          <div className="col col--4">Feature 3</div>
        </div>
      </div>
    </section>
  );
}

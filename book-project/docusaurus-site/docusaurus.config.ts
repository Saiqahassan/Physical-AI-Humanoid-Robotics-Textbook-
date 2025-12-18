import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building the Future of Robotics',
  favicon: 'img/favicon.ico',

  url: 'https://physical-ai-humanoid-robotics-textbook.vercel.app',
  baseUrl: '/',

  organizationName: 'Saiqahassan',
  projectName: 'Physical-AI-Humanoid-Robotics-Textbook-',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/Saiqahassan/Physical-AI-Humanoid-Robotics-Textbook-/tree/main/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/Saiqahassan/Physical-AI-Humanoid-Robotics-Textbook-/tree/main/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        { to: '/blog', label: 'Blog', position: 'left' },
        {
          href: 'https://github.com/Saiqahassan/Physical-AI-Humanoid-Robotics-Textbook-',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [{ label: 'Book', to: '/docs/intro' }],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'X',
              href: 'https://x.com',
            },
          ],
        },
        {
          title: 'More',
          items: [
            { label: 'Blog', to: '/blog' },
            {
              label: 'GitHub',
              href: 'https://github.com/Saiqahassan/Physical-AI-Humanoid-Robotics-Textbook-',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

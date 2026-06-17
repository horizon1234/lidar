# -*- coding: utf-8 -*-
"""
将 docs/ 下的 0~21 号 Markdown 文档转换为 PDF。

设计要点：
- 数学公式：通过 KaTeX 在浏览器端渲染（支持 $...$ 与 $$...$$），
  由 pymdownx.arithmatex 把公式提取为 <span class="arithmatex">...</span>，
  再由 KaTeX 的 auto-render 自动识别并渲染。
- 中文：使用系统中文字体（Windows 自带「微软雅黑」），无需额外字体文件。
- PDF 生成：使用系统自带的 Microsoft Edge（Chromium 内核）headless 模式
  将渲染好的 HTML 打印成 A4 PDF，对中文字形与数学公式支持最好。
- 排版：A4 纸，舒适的正文行距、页边距、标题层级与代码块样式。
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

import markdown


# --------------------------------------------------------------------------- #
# 路径配置
# --------------------------------------------------------------------------- #
ROOT = Path(__file__).resolve().parents[1]  # 项目根目录
DOCS_DIR = ROOT / "docs"
PDF_DIR = DOCS_DIR / "pdf"

# Edge 可执行文件候选位置（Windows）
EDGE_CANDIDATES = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]
CHROME_CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]


# --------------------------------------------------------------------------- #
# HTML 模板
# --------------------------------------------------------------------------- #
HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>{title}</title>
<link rel="stylesheet"
      href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">
<script defer
        src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>
<script defer
        src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js"
        onload="renderMathInElement(document.body, {{
            delimiters: [
                {{left: '$$$$', right: '$$$$', display: true}},
                {{left: '$',    right: '$',    display: false}},
                {{left: '\\\\(', right: '\\\\)', display: false}},
                {{left: '\\\\[', right: '\\\\]', display: true}}
            ],
            throwOnError: false
        }});"></script>
<style>
  @page {{
    size: A4;
    margin: 18mm 16mm 18mm 16mm;
  }}
  html {{ -webkit-print-color-adjust: exact; }}
  body {{
    font-family: "Microsoft YaHei", "微软雅黑", "PingFang SC",
                 "Hiragino Sans GB", "Source Han Sans SC", "Noto Sans CJK SC",
                 Arial, sans-serif;
    font-size: 11pt;
    line-height: 1.75;
    color: #1f2328;
    max-width: 820px;
    margin: 0 auto;
    padding: 4px 0 24px 0;
  }}
  h1, h2, h3, h4, h5, h6 {{
    font-weight: 600;
    line-height: 1.3;
    margin: 1.6em 0 0.6em;
    color: #0b0d12;
  }}
  h1 {{
    font-size: 1.8em;
    border-bottom: 2px solid #d0d7de;
    padding-bottom: 0.3em;
    margin-top: 0.2em;
  }}
  h2 {{
    font-size: 1.45em;
    border-bottom: 1px solid #e1e4e8;
    padding-bottom: 0.25em;
  }}
  h3 {{ font-size: 1.2em; }}
  h4 {{ font-size: 1.05em; }}
  p {{ margin: 0.7em 0; }}
  a {{ color: #0969da; text-decoration: none; }}
  ul, ol {{ padding-left: 1.6em; margin: 0.6em 0; }}
  li {{ margin: 0.25em 0; }}
  blockquote {{
    margin: 0.8em 0;
    padding: 0.4em 1em;
    border-left: 4px solid #d0d7de;
    color: #57606a;
    background: #f6f8fa;
    border-radius: 0 4px 4px 0;
  }}
  code {{
    font-family: "Cascadia Code", "Consolas", "Courier New", monospace;
    font-size: 0.9em;
    background: #f1f3f5;
    padding: 0.12em 0.4em;
    border-radius: 4px;
    color: #b01a3b;
  }}
  pre {{
    background: #f6f8fa;
    border: 1px solid #e1e4e8;
    border-radius: 6px;
    padding: 0.9em 1em;
    overflow-x: auto;
    line-height: 1.5;
    font-size: 0.88em;
  }}
  pre code {{
    background: transparent;
    padding: 0;
    color: #1f2328;
  }}
  table {{
    border-collapse: collapse;
    margin: 1em 0;
    width: 100%;
    font-size: 0.95em;
  }}
  th, td {{
    border: 1px solid #d0d7de;
    padding: 6px 12px;
    text-align: left;
  }}
  th {{ background: #f6f8fa; font-weight: 600; }}
  tr:nth-child(even) td {{ background: #fbfcfd; }}
  hr {{
    border: none;
    border-top: 1px solid #d0d7de;
    margin: 1.6em 0;
  }}
  img {{ max-width: 100%; }}
  .arithmatex {{ display: inline; }}
  .math-display {{ display: block; margin: 0.8em 0; }}
</style>
</head>
<body>
{body}
</body>
</html>
"""


# --------------------------------------------------------------------------- #
# 工具函数
# --------------------------------------------------------------------------- #
def find_chromium() -> str:
    """返回系统上可用的 Chromium 内核浏览器路径。"""
    for path in EDGE_CANDIDATES + CHROME_CANDIDATES:
        if Path(path).exists():
            return path
    raise RuntimeError(
        "未找到 Microsoft Edge 或 Google Chrome，无法生成 PDF。"
    )


def md_extensions() -> list[str]:
    """pymdownx 的扩展配置，arithmatex 让 KaTeX 渲染公式。"""
    return [
        "extra",
        "sane_lists",
        "toc",
        "nl2br",
        "pymdownx.arithmatex",
        "pymdownx.superfences",
        "pymdownx.tasklist",
        "pymdownx.tilde",
        "pymdownx.mark",
        "pymdownx.magiclink",
    ]


def md_extension_configs() -> dict:
    return {
        "pymdownx.arithmatex": {
            "generic": True,
            "preview": False,
        },
        "toc": {"permalink": False},
        "pymdownx.superfences": {"disable_indented_code_blocks": False},
    }


def natural_sort_key(name: str):
    """对 '0. ', '1. ', ..., '21 ', '3.5 ' 这类前缀做自然排序。"""
    m = re.match(r"^\s*(\d+(?:\.\d+)?)\.?\s+", name)
    if m:
        num = float(m.group(1))
        return (0, num, name)
    return (1, 0.0, name)


def convert_md_to_html(md_path: Path) -> str:
    text = md_path.read_text(encoding="utf-8")
    html_body = markdown.markdown(
        text,
        extensions=md_extensions(),
        extension_configs=md_extension_configs(),
        output_format="html5",
    )
    title = md_path.stem
    return HTML_TEMPLATE.format(title=title, body=html_body)


def html_to_pdf(chromium: str, html_path: Path, pdf_path: Path) -> None:
    """调用 Chromium headless 将 HTML 打印成 PDF。"""
    cmd = [
        chromium,
        "--headless=new",
        "--disable-gpu",
        "--no-pdf-header-footer",
        "--run-all-compositor-stages-before-draw",
        "--virtual-time-budget=8000",  # 等待 KaTeX 渲染完成
        f"--print-to-pdf={pdf_path}",
        html_path.as_uri(),
    ]
    subprocess.run(cmd, check=True, capture_output=True)


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def select_target_files() -> list[Path]:
    """挑选 docs 下文件名以 0~21 开头的 .md 文件（自然排序）。"""
    md_files = sorted(DOCS_DIR.glob("*.md"), key=lambda p: natural_sort_key(p.name))
    targets = []
    for p in md_files:
        m = re.match(r"^(\d+(?:\.\d+)?)\.?\s", p.name)
        if m and 0 <= float(m.group(1)) < 21:
            targets.append(p)
    return targets


def make_safe_pdf_name(md_path: Path) -> str:
    """生成稳定的 PDF 文件名：保留原编号前缀，替换掉 Windows 非法字符。"""
    name = md_path.stem
    name = re.sub(r'[\\/:*?"<>|]', "_", name).strip()
    return f"{name}.pdf"


def main() -> int:
    if PDF_DIR.exists():
        shutil.rmtree(PDF_DIR)
    PDF_DIR.mkdir(parents=True, exist_ok=True)

    try:
        chromium = find_chromium()
    except RuntimeError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 2

    print(f"[INFO] 使用浏览器: {chromium}")
    print(f"[INFO] 输出目录  : {PDF_DIR}")

    targets = select_target_files()
    if not targets:
        print("[WARN] 没有找到 0~21 开头的 Markdown 文件。")
        return 1

    print(f"[INFO] 共发现 {len(targets)} 个文件待转换。\n")

    tmp_html_dir = PDF_DIR / "_html_tmp"
    tmp_html_dir.mkdir(exist_ok=True)

    ok, fail = 0, 0
    for md_path in targets:
        pdf_name = make_safe_pdf_name(md_path)
        pdf_path = PDF_DIR / pdf_name
        html_path = tmp_html_dir / f"{md_path.stem}.html"
        try:
            html = convert_md_to_html(md_path)
            html_path.write_text(html, encoding="utf-8")
            html_to_pdf(chromium, html_path, pdf_path)
            ok += 1
            print(f"  [OK]   {md_path.name}  ->  pdf/{pdf_name}")
        except subprocess.CalledProcessError as e:
            fail += 1
            err = e.stderr.decode("utf-8", errors="ignore")[:300] if e.stderr else ""
            print(f"  [FAIL] {md_path.name} : {err}")
        except Exception as e:  # noqa: BLE001
            fail += 1
            print(f"  [FAIL] {md_path.name} : {e}")

    # 清理临时 HTML
    shutil.rmtree(tmp_html_dir, ignore_errors=True)

    print(f"\n[DONE] 成功 {ok} 个，失败 {fail} 个。")
    print(f"       PDF 已保存到: {PDF_DIR}")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
